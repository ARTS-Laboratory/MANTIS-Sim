function [Y, tOut, meta, cleanY, distances, delays, doppler] = ...
    simulate_uav_acoustics(micXY, uavXYZ, tTrajectory, settings)
%SIMULATE_UAV_ACOUSTICS Simulate UAV acoustic signals at planar microphones.
%
%   [Y, t, meta] = SIMULATE_UAV_ACOUSTICS(micXY, uavXYZ, tTrajectory)
%   [Y, t, meta, cleanY, distances, delays, doppler] = ...
%       SIMULATE_UAV_ACOUSTICS(micXY, uavXYZ, tTrajectory, settings)
%
% Inputs
%   micXY        - N-by-2 microphone coordinates in metres, z = 0
%   uavXYZ       - T-by-3 UAV coordinates in metres
%   tTrajectory  - T-by-1 strictly increasing trajectory times in seconds
%   settings     - scalar structure; see UAV_ACOUSTIC_DEFAULTS
%
% Outputs
%   Y            - numSamples-by-N noisy signals, with optional dropout
%   tOut         - uniformly sampled acoustic output time vector
%   meta         - geometry, source, noise, propagation, and run metadata
%   cleanY       - propagated signals before ambient noise and dropout
%   distances    - receiver-time source-to-microphone distances, metres
%   delays       - distances / speed of sound, seconds
%   doppler      - structure containing radial velocity, ratio, and shift Hz
%
% Calling SIMULATE_UAV_ACOUSTICS() uses a five-microphone layout and a
% straight default path. Calling SIMULATE_UAV_ACOUSTICS(settings) is also
% supported when the only input is a settings structure.

if nargin == 1 && isstruct(micXY) && isscalar(micXY)
    settings = micXY;
    micXY = [];
    uavXYZ = [];
    tTrajectory = [];
elseif nargin < 4 || isempty(settings)
    settings = struct();
end
if nargin < 3
    tTrajectory = [];
end
if nargin < 2
    uavXYZ = [];
end
if nargin < 1
    micXY = [];
end

settings = uav_acoustic_defaults(settings);
local_validate_core_settings(settings);

usedDefaultMicrophones = isempty(micXY);
if usedDefaultMicrophones
    micXY = [-50 -50; 50 -50; 0 0; -50 50; 50 50];
end
local_validate_microphones(micXY);

usedDefaultTrajectory = isempty(uavXYZ);
if usedDefaultTrajectory
    if isempty(tTrajectory)
        trajectoryParams = struct('startXYZ', [-140 -70 80], ...
            'endXYZ', [140 70 80]);
        [uavXYZ, tTrajectory] = generate_uav_trajectory('straight', ...
            settings.defaultDurationSec, settings.defaultTrajectoryRateHz, ...
            trajectoryParams);
    else
        tTrajectory = local_validate_time_vector(tTrajectory);
        fraction = (tTrajectory - tTrajectory(1)) / ...
            (tTrajectory(end) - tTrajectory(1));
        startXYZ = [-140 -70 80];
        endXYZ = [140 70 80];
        uavXYZ = startXYZ + fraction .* (endXYZ - startXYZ);
    end
elseif isempty(tTrajectory)
    error('simulate_uav_acoustics:MissingTrajectoryTime', ...
        'tTrajectory is required when uavXYZ is supplied.');
end

[tTrajectory, uavXYZ] = local_validate_trajectory(tTrajectory, uavXYZ);

% Build a strictly uniform acoustic time vector. The final sample is the
% last fs-spaced sample that does not exceed the trajectory end time.
durationSec = tTrajectory(end) - tTrajectory(1);
numSamples = floor(durationSec * settings.fs + 10 * eps(durationSec * settings.fs)) + 1;
if numSamples < 2
    error('simulate_uav_acoustics:TrajectoryTooShort', ...
        'The trajectory duration must be at least one acoustic sample period.');
end
tOut = tTrajectory(1) + (0:numSamples - 1).' / settings.fs;

trajectoryMethod = lower(strtrim(local_to_char( ...
    settings.trajectoryInterpolation, 'settings.trajectoryInterpolation')));
validTrajectoryMethods = {'linear', 'nearest', 'pchip', 'spline', 'makima'};
if ~any(strcmpi(trajectoryMethod, validTrajectoryMethods))
    error('simulate_uav_acoustics:InvalidTrajectoryInterpolation', ...
        'trajectoryInterpolation must be linear, nearest, pchip, spline, or makima.');
end
if strcmpi(trajectoryMethod, 'pchip') && numel(tTrajectory) < 4
    warning('simulate_uav_acoustics:PchipNeedsMorePoints', ...
        ['pchip trajectory interpolation needs at least four points; ' ...
        'using linear interpolation for this trajectory.']);
    trajectoryMethod = 'linear';
end
sampledUavXYZ = interp1(tTrajectory, uavXYZ, tOut, trajectoryMethod);

% Catch physically unreasonable inputs early. The model remains stable for
% ordinary UAV speeds and clips only diagnostic Doppler ratios.
sampledSpeed = local_speed(sampledUavXYZ, settings.fs);
maxSpeedMps = max(sampledSpeed);
if maxSpeedMps >= 0.95 * settings.c
    error('simulate_uav_acoustics:SupersonicTrajectory', ...
        'The supplied trajectory reaches %.3f m/s, too close to or above c.', ...
        maxSpeedMps);
elseif maxSpeedMps > 0.30 * settings.c
    warning('simulate_uav_acoustics:VeryFastTrajectory', ...
        ['The trajectory reaches %.3f m/s. The simplified propagation model ' ...
        'is intended primarily for subsonic UAV motion.'], maxSpeedMps);
end

% Keep the run repeatable without permanently changing the caller's random
% stream. Random source components, noise, and dropout all share this seed.
previousRandomState = rng;
restoreRandomState = onCleanup(@() rng(previousRandomState)); %#ok<NASGU>
rng(settings.randomSeed, 'twister');

[sourceSignal, sourceMeta] = generate_source_signal(tOut, settings.source);
propagationSettings = settings;
propagationSettings.sourceReferenceFrequencyHz = sourceMeta.referenceFrequencyHz;
[cleanY, distances, delays, doppler, propagationInfo] = ...
    propagate_to_microphones(sourceSignal, tOut, micXY, sampledUavXYZ, ...
    propagationSettings);
[YwithNoise, ~, noiseInfo] = add_noise(cleanY, settings.snrDb, ...
    settings.noise, settings.fs);
[Y, dropoutMask, dropoutInfo] = apply_sensor_dropout(YwithNoise, ...
    settings.dropout, settings.fs);

% Package metadata while avoiding duplicate storage of a user waveform.
meta = struct();
meta.toolboxName = 'MATLAB UAV Acoustic Emulator';
meta.version = '1.0.0';
meta.createdAt = datestr(now, 30);
meta.fs = settings.fs;
meta.c = settings.c;
meta.alpha = settings.alpha;
meta.snrDb = settings.snrDb;
meta.randomSeed = settings.randomSeed;
meta.numSamples = numSamples;
meta.numMics = size(micXY, 1);
meta.durationSec = tOut(end) - tOut(1);
meta.simulationTime = struct('startSec', tOut(1), 'endSec', tOut(end), ...
    'durationSec', meta.durationSec);
meta.micXY = micXY;
meta.microphoneZ = 0;
meta.uavTrajectory = struct();
meta.uavTrajectory.inputTime = tTrajectory;
meta.uavTrajectory.inputXYZ = uavXYZ;
if settings.metadata.storeSampledTrajectory
    meta.uavTrajectory.sampledTime = tOut;
    meta.uavTrajectory.sampledXYZ = sampledUavXYZ;
end
meta.uavTrajectory.maxSpeedMps = maxSpeedMps;
meta.source = sourceMeta;
meta.propagation = propagationInfo;
meta.noise = noiseInfo;
meta.dropout = dropoutInfo;
if settings.dropout.storeMask
    meta.dropout.mask = dropoutMask;
end
meta.effects = struct('movingDelay', true, ...
    'doppler', logical(settings.enableDoppler), ...
    'dopplerRefinement', logical(settings.enableDoppler), ...
    'reflection', logical(settings.enableReflection), ...
    'dropout', settings.dropout.rate > 0);
meta.defaultsUsed = struct('microphones', usedDefaultMicrophones, ...
    'trajectory', usedDefaultTrajectory);
meta.output = struct('signalSize', size(Y), ...
    'cleanSignalAvailable', true, ...
    'distanceSize', size(distances), ...
    'delaySize', size(delays));
meta.units = struct('position', 'm', 'time', 's', 'sampleRate', 'Hz', ...
    'radialVelocity', 'm/s', 'signalAmplitude', 'arbitrary pressure units');
meta.dopplerSummary = local_doppler_summary(doppler);
if settings.metadata.storeSettings
    meta.settings = local_sanitize_settings(settings);
end

% Defensive dimensional checks make downstream failures easier to diagnose.
expectedSize = [numSamples size(micXY, 1)];
if ~isequal(size(Y), expectedSize) || ~isequal(size(cleanY), expectedSize) ...
        || ~isequal(size(distances), expectedSize) ...
        || ~isequal(size(delays), expectedSize) ...
        || ~isequal(size(doppler.ratio), expectedSize)
    error('simulate_uav_acoustics:InternalDimensionMismatch', ...
        'An internal output dimension is inconsistent with the geometry.');
end
end

function local_validate_core_settings(settings)
local_validate_positive_scalar(settings.fs, 'settings.fs');
local_validate_positive_scalar(settings.c, 'settings.c');
local_validate_positive_scalar(settings.alpha, 'settings.alpha');
if ~isnumeric(settings.snrDb) || ~isreal(settings.snrDb) ...
        || ~isscalar(settings.snrDb) || isnan(settings.snrDb) ...
        || settings.snrDb == -Inf
    error('simulate_uav_acoustics:InvalidSNR', ...
        'settings.snrDb must be a finite scalar or positive Inf.');
end
if ~isnumeric(settings.randomSeed) || ~isreal(settings.randomSeed) ...
        || ~isscalar(settings.randomSeed) || ~isfinite(settings.randomSeed) ...
        || settings.randomSeed < 0 || settings.randomSeed > 2^32 - 1 ...
        || settings.randomSeed ~= floor(settings.randomSeed)
    error('simulate_uav_acoustics:InvalidRandomSeed', ...
        'settings.randomSeed must be an integer scalar in [0, 2^32-1].');
end
local_validate_positive_scalar(settings.defaultDurationSec, ...
    'settings.defaultDurationSec');
local_validate_positive_scalar(settings.defaultTrajectoryRateHz, ...
    'settings.defaultTrajectoryRateHz');
local_validate_logical_scalar(settings.enableDoppler, 'settings.enableDoppler');
local_validate_logical_scalar(settings.enableReflection, 'settings.enableReflection');
local_validate_logical_scalar(settings.metadata.storeSampledTrajectory, ...
    'settings.metadata.storeSampledTrajectory');
local_validate_logical_scalar(settings.metadata.storeSettings, ...
    'settings.metadata.storeSettings');
local_validate_logical_scalar(settings.dropout.storeMask, ...
    'settings.dropout.storeMask');
end

function local_validate_microphones(micXY)
if ~isnumeric(micXY) || ~isreal(micXY) || ndims(micXY) ~= 2 ...
        || size(micXY, 2) ~= 2 || isempty(micXY) || any(~isfinite(micXY(:)))
    error('simulate_uav_acoustics:InvalidMicrophones', ...
        'micXY must be a nonempty, finite N-by-2 numeric array.');
end
end

function [t, xyz] = local_validate_trajectory(t, xyz)
t = local_validate_time_vector(t);
if ~isnumeric(xyz) || ~isreal(xyz) || ndims(xyz) ~= 2 ...
        || size(xyz, 2) ~= 3 || size(xyz, 1) ~= numel(t) ...
        || any(~isfinite(xyz(:)))
    error('simulate_uav_acoustics:InvalidTrajectory', ...
        'uavXYZ must be a finite T-by-3 numeric array matching tTrajectory.');
end
if any(xyz(:, 3) < 0)
    error('simulate_uav_acoustics:NegativeAltitude', ...
        'UAV altitude must be nonnegative because microphones are at z = 0.');
end
end

function t = local_validate_time_vector(t)
if ~isnumeric(t) || ~isreal(t) || ~isvector(t) || numel(t) < 2 ...
        || any(~isfinite(t(:))) || any(diff(t(:)) <= 0)
    error('simulate_uav_acoustics:InvalidTrajectoryTime', ...
        'tTrajectory must be finite, strictly increasing, and contain at least two samples.');
end
t = t(:);
end

function speed = local_speed(xyz, fs)
velocity = [gradient(xyz(:, 1), 1 / fs), ...
    gradient(xyz(:, 2), 1 / fs), gradient(xyz(:, 3), 1 / fs)];
speed = sqrt(sum(velocity .^ 2, 2));
end

function summary = local_doppler_summary(doppler)
summary = struct();
summary.model = doppler.model;
summary.referenceFrequencyHz = doppler.referenceFrequencyHz;
summary.radialVelocityRangeMps = [min(doppler.radialVelocityMps(:)), ...
    max(doppler.radialVelocityMps(:))];
summary.ratioRange = [min(doppler.ratio(:)), max(doppler.ratio(:))];
finiteShift = doppler.shiftHz(isfinite(doppler.shiftHz));
if isempty(finiteShift)
    summary.shiftRangeHz = [NaN NaN];
else
    summary.shiftRangeHz = [min(finiteShift), max(finiteShift)];
end
end

function settings = local_sanitize_settings(settings)
if isfield(settings, 'source')
    if isfield(settings.source, 'userSignal') && ~isempty(settings.source.userSignal)
        settings.source.userSignalLength = numel(settings.source.userSignal);
        settings.source.userSignal = [];
    end
    if isfield(settings.source, 'userTime') && ~isempty(settings.source.userTime)
        settings.source.userTimeLength = numel(settings.source.userTime);
        settings.source.userTime = [];
    end
end
end

function text = local_to_char(value, name)
if isstring(value)
    if ~isscalar(value)
        error('simulate_uav_acoustics:InvalidText', '%s must be scalar text.', name);
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('simulate_uav_acoustics:InvalidText', ...
        '%s must be a character vector or scalar string.', name);
end
end

function local_validate_positive_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value <= 0
    error('simulate_uav_acoustics:InvalidPositiveScalar', ...
        '%s must be a positive finite numeric scalar.', name);
end
end

function local_validate_logical_scalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) ...
        || ~isfinite(double(value)) || ~(value == 0 || value == 1)
    error('simulate_uav_acoustics:InvalidLogical', ...
        '%s must be a logical scalar.', name);
end
end
