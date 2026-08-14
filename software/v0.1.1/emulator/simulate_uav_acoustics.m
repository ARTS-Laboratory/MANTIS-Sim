function [Y, t, meta, extras] = simulate_uav_acoustics(micXY, uavXYZ, t, varargin)
%SIMULATE_UAV_ACOUSTICS Main entry point for the UAV acoustic emulator.
%
%   [Y, t, meta] = SIMULATE_UAV_ACOUSTICS(micXY, uavXYZ, t) simulates the
%   acoustic signal received at each of a set of ground microphones as a
%   UAV flies overhead, and returns the resulting time-series matrix.
%
%   [Y, t, meta, extras] = SIMULATE_UAV_ACOUSTICS(...) additionally
%   returns a struct EXTRAS with optional diagnostic outputs (clean
%   signal, per-mic distances, delays, and Doppler estimates).
%
% REQUIRED INPUTS
%   micXY   : N-by-2 matrix of microphone (x,y) coordinates in meters.
%             Microphones are assumed to lie on the z = 0 plane.
%   uavXYZ  : T-by-3 matrix of UAV (x,y,z) positions in meters, sampled
%             at the times given in t.
%   t       : 1-by-T or T-by-1 vector of simulation times in seconds,
%             must be monotonically increasing and match size(uavXYZ,1).
%
% OPTIONAL NAME-VALUE PAIRS
%   'fs'              sampling rate in Hz (default: inferred from t, or
%                      8000 if t has fewer than 2 samples)
%   'c'                speed of sound in m/s (default: 343)
%   'alpha'            attenuation exponent, 1 = inverse distance,
%                      2 = inverse distance squared (default: 1)
%   'snrDb'            desired signal-to-noise ratio in dB (default: 15)
%   'enableDoppler'    true/false (default: false)
%   'enableReflection' true/false (default: false)
%   'reflectionGain'   relative amplitude of the ground-reflected arrival
%                      as a fraction of the direct path (default: 0.4)
%   'sourceType'       'tone' | 'multitone' | 'rotor' | 'noiseburst' |
%                      'custom' (default: 'rotor')
%   'sourceFreq'       fundamental source frequency in Hz (default: 120)
%   'sourceWaveform'   user-supplied source waveform vector, required
%                      when sourceType = 'custom'
%   'noiseType'        'white' | 'pink' | 'wind' (default: 'white')
%   'dropoutRate'      probability in [0,1] that a given microphone
%                      channel is entirely dropped out / dead (default: 0)
%   'seed'             random seed for reproducibility (default: 42)
%
% OUTPUTS
%   Y       : numSamples-by-numMics matrix of simulated microphone
%             signals (noisy).
%   t       : the simulation time vector actually used (column vector).
%   meta    : struct with microphone coordinates, UAV trajectory, source
%             settings, attenuation parameters, noise settings,
%             simulation time, and enabled effects.
%   extras  : struct with optional fields cleanY, distances, delays,
%             doppler (each numSamples/T-by-numMics as appropriate).
%
% EXAMPLE
%   micXY  = [0 0; 50 0; 0 50; -50 0; 0 -50];
%   t      = (0:1/2000:10)';
%   uavXYZ = [linspace(-200,200,numel(t))', zeros(numel(t),1), ...
%             80*ones(numel(t),1)];
%   [Y, t, meta] = simulate_uav_acoustics(micXY, uavXYZ, t, ...
%       'enableDoppler', true, 'snrDb', 10);
%
% See also GENERATE_UAV_TRAJECTORY, GENERATE_SOURCE_SIGNAL,
%          PROPAGATE_TO_MICROPHONES, ADD_NOISE, SAVE_SIMULATION,
%          PLOT_SIMULATION_RESULTS

% ---------------------------------------------------------------------
% 1. Parse and validate inputs
% ---------------------------------------------------------------------
p = inputParser;
p.FunctionName = 'simulate_uav_acoustics';

addRequired(p, 'micXY');
addRequired(p, 'uavXYZ');
addRequired(p, 't');

addParameter(p, 'fs', []);
addParameter(p, 'c', 343);
addParameter(p, 'alpha', 1);
addParameter(p, 'snrDb', 15);
addParameter(p, 'enableDoppler', false);
addParameter(p, 'enableReflection', false);
addParameter(p, 'reflectionGain', 0.4);
addParameter(p, 'sourceType', 'rotor');
addParameter(p, 'sourceFreq', 120);
addParameter(p, 'sourceWaveform', []);
addParameter(p, 'noiseType', 'white');
addParameter(p, 'dropoutRate', 0);
addParameter(p, 'seed', 42);

parse(p, micXY, uavXYZ, t, varargin{:});
opt = p.Results;

t = t(:); % force column vector

% --- geometry / dimension checks -------------------------------------
if isempty(micXY) || size(micXY,2) ~= 2
    error('simulate_uav_acoustics:badMicXY', ...
        'micXY must be an N-by-2 matrix of (x,y) microphone coordinates.');
end
if ~isnumeric(micXY) || any(~isfinite(micXY(:)))
    error('simulate_uav_acoustics:badMicXY', ...
        'micXY must contain finite numeric values.');
end

if isempty(uavXYZ) || size(uavXYZ,2) ~= 3
    error('simulate_uav_acoustics:badUavXYZ', ...
        'uavXYZ must be a T-by-3 matrix of (x,y,z) UAV positions.');
end
if ~isnumeric(uavXYZ) || any(~isfinite(uavXYZ(:)))
    error('simulate_uav_acoustics:badUavXYZ', ...
        'uavXYZ must contain finite numeric values.');
end

if size(uavXYZ,1) ~= numel(t)
    error('simulate_uav_acoustics:sizeMismatch', ...
        'Number of rows in uavXYZ (%d) must match numel(t) (%d).', ...
        size(uavXYZ,1), numel(t));
end
if numel(t) < 2
    error('simulate_uav_acoustics:tooFewSamples', ...
        't must contain at least 2 samples.');
end
if any(diff(t) <= 0)
    error('simulate_uav_acoustics:tNotMonotonic', ...
        't must be strictly increasing.');
end

% --- fs: infer from t if not supplied ---------------------------------
if isempty(opt.fs)
    dt = median(diff(t));
    fs = 1/dt;
else
    fs = opt.fs;
end
if ~isscalar(fs) || ~isfinite(fs) || fs <= 0
    error('simulate_uav_acoustics:badFs', 'fs must be a positive finite scalar.');
end

% --- other scalar checks ----------------------------------------------
if ~isscalar(opt.c) || ~isfinite(opt.c) || opt.c <= 0
    error('simulate_uav_acoustics:badC', 'c (speed of sound) must be a positive finite scalar.');
end
if ~isscalar(opt.alpha) || ~isfinite(opt.alpha) || opt.alpha < 0
    error('simulate_uav_acoustics:badAlpha', 'alpha must be a non-negative finite scalar.');
end
if ~isscalar(opt.snrDb) || ~isfinite(opt.snrDb)
    error('simulate_uav_acoustics:badSnr', 'snrDb must be a finite numeric scalar.');
end
if opt.dropoutRate < 0 || opt.dropoutRate > 1
    error('simulate_uav_acoustics:badDropout', 'dropoutRate must be in [0,1].');
end

numMics = size(micXY, 1);
numSamples = numel(t);

% ---------------------------------------------------------------------
% 2. Reproducibility
% ---------------------------------------------------------------------
rng(opt.seed);

% ---------------------------------------------------------------------
% 3. Generate source waveform
% ---------------------------------------------------------------------
sourceOpts = struct( ...
    'type', opt.sourceType, ...
    'freq', opt.sourceFreq, ...
    'customWaveform', opt.sourceWaveform);
s = generate_source_signal(t, fs, sourceOpts);

% ---------------------------------------------------------------------
% 4. Propagate to each microphone (delay, attenuation, Doppler, reflection)
% ---------------------------------------------------------------------
propOpts = struct( ...
    'c', opt.c, ...
    'alpha', opt.alpha, ...
    'enableDoppler', opt.enableDoppler, ...
    'enableReflection', opt.enableReflection, ...
    'reflectionGain', opt.reflectionGain);

[cleanY, distances, delays, dopplerShift] = propagate_to_microphones( ...
    micXY, uavXYZ, t, fs, s, propOpts);

% ---------------------------------------------------------------------
% 5. Apply sensor dropout (missing-node simulation)
% ---------------------------------------------------------------------
deadMics = false(1, numMics);
if opt.dropoutRate > 0
    deadMics = rand(1, numMics) < opt.dropoutRate;
    cleanY(:, deadMics) = 0;
end

% ---------------------------------------------------------------------
% 6. Add noise
% ---------------------------------------------------------------------
noiseOpts = struct('snrDb', opt.snrDb, 'type', opt.noiseType);
Y = add_noise(cleanY, noiseOpts);
% Dead microphones stay silent (noise-free flatline) to make dropout
% unambiguous to downstream algorithms.
Y(:, deadMics) = 0;

% ---------------------------------------------------------------------
% 7. Package metadata and optional extras
% ---------------------------------------------------------------------
meta = struct();
meta.micXY = micXY;
meta.uavXYZ = uavXYZ;
meta.t = t;
meta.fs = fs;
meta.numMics = numMics;
meta.numSamples = numSamples;
meta.source = sourceOpts;
meta.attenuation = struct('alpha', opt.alpha);
meta.propagation = struct('c', opt.c);
meta.noise = noiseOpts;
meta.effects = struct( ...
    'doppler', opt.enableDoppler, ...
    'reflection', opt.enableReflection, ...
    'reflectionGain', opt.reflectionGain);
meta.dropout = struct('rate', opt.dropoutRate, 'deadMics', deadMics);
meta.seed = opt.seed;
meta.createdAt = datestr(now);

extras = struct();
extras.cleanY = cleanY;
extras.distances = distances;
extras.delays = delays;
extras.doppler = dopplerShift;

end
