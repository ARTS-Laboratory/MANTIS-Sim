function [sourceSignal, meta] = generate_source_signal(t, sourceSettings)
%GENERATE_SOURCE_SIGNAL Create a repeatable UAV-like acoustic source signal.
%
%   [s, meta] = GENERATE_SOURCE_SIGNAL(t, sourceSettings)
%
% sourceSettings.type can be:
%   'tone'        - one sinusoid
%   'tone-noise'  - one tone mixed with colored broadband noise
%   'multitone'   - an arbitrary list of sinusoidal components
%   'rotor'       - harmonic rotor model with slow amplitude modulation
%   'broadband'   - colored-noise burst or continuous colored noise
%   'user'        - a user-supplied waveform
%
% Random components use the caller's current random-number stream. The main
% simulator sets and restores that stream using settings.randomSeed.

if nargin < 1 || isempty(t)
    error('generate_source_signal:MissingTimeVector', ...
        'A nonempty time vector is required.');
end
if nargin < 2 || isempty(sourceSettings)
    sourceSettings = struct();
end

if ~isnumeric(t) || ~isreal(t) || ~isvector(t) || numel(t) < 2 ...
        || any(~isfinite(t(:))) || any(diff(t(:)) <= 0)
    error('generate_source_signal:InvalidTimeVector', ...
        't must be a finite, strictly increasing numeric vector.');
end
if ~isstruct(sourceSettings) || ~isscalar(sourceSettings)
    error('generate_source_signal:InvalidSettings', ...
        'sourceSettings must be a scalar structure.');
end

t = t(:);
dt = median(diff(t));
fs = 1 / dt;
tRelative = t - t(1);
numSamples = numel(t);

allSettings = uav_acoustic_defaults(struct('source', sourceSettings));
settings = allSettings.source;
sourceType = local_to_char(settings.type, 'sourceSettings.type');
sourceType = lower(strtrim(sourceType));

local_validate_nonnegative_scalar(settings.amplitudeRms, ...
    'sourceSettings.amplitudeRms');
local_validate_logical_scalar(settings.removeMean, ...
    'sourceSettings.removeMean');

referenceFrequencyHz = NaN;
componentFrequenciesHz = [];
raw = zeros(numSamples, 1);

switch sourceType
    case {'tone', 'single-tone', 'single_tone'}
        frequencyHz = settings.frequencyHz;
        phaseRad = settings.phaseRad;
        local_validate_frequency(frequencyHz, fs, 'sourceSettings.frequencyHz');
        local_validate_finite_scalar(phaseRad, 'sourceSettings.phaseRad');
        raw = sin(2 * pi * frequencyHz * tRelative + phaseRad);
        referenceFrequencyHz = frequencyHz;
        componentFrequenciesHz = frequencyHz;

    case {'tone-noise', 'tone+noise', 'tone-plus-noise', 'tone_noise'}
        frequencyHz = settings.frequencyHz;
        phaseRad = settings.phaseRad;
        broadbandFraction = settings.broadbandFraction;
        local_validate_frequency(frequencyHz, fs, 'sourceSettings.frequencyHz');
        local_validate_finite_scalar(phaseRad, 'sourceSettings.phaseRad');
        local_validate_fraction(broadbandFraction, ...
            'sourceSettings.broadbandFraction');
        tonal = sin(2 * pi * frequencyHz * tRelative + phaseRad);
        broadband = local_colored_noise(numSamples, settings.noiseColor, fs);
        raw = sqrt(1 - broadbandFraction) * local_unit_rms(tonal) + ...
            sqrt(broadbandFraction) * local_unit_rms(broadband);
        referenceFrequencyHz = frequencyHz;
        componentFrequenciesHz = frequencyHz;

    case {'multitone', 'multi-tone', 'multi_tone'}
        frequenciesHz = settings.frequenciesHz(:).';
        relativeAmplitudes = settings.relativeAmplitudes(:).';
        if isempty(frequenciesHz) || any(~isfinite(frequenciesHz)) ...
                || any(frequenciesHz <= 0) || any(frequenciesHz >= fs / 2)
            error('generate_source_signal:InvalidFrequencies', ...
                'All multitone frequencies must be in the open interval (0, fs/2).');
        end
        if numel(relativeAmplitudes) == 1 && numel(frequenciesHz) > 1
            relativeAmplitudes = repmat(relativeAmplitudes, size(frequenciesHz));
        end
        if numel(relativeAmplitudes) ~= numel(frequenciesHz) ...
                || any(~isfinite(relativeAmplitudes))
            error('generate_source_signal:InvalidAmplitudes', ...
                ['sourceSettings.relativeAmplitudes must be scalar or match ' ...
                'sourceSettings.frequenciesHz.']);
        end
        phasesRad = settings.phasesRad;
        if isempty(phasesRad)
            phasesRad = 0.31 * (0:numel(frequenciesHz) - 1);
        else
            phasesRad = phasesRad(:).';
        end
        if numel(phasesRad) == 1 && numel(frequenciesHz) > 1
            phasesRad = repmat(phasesRad, size(frequenciesHz));
        end
        if numel(phasesRad) ~= numel(frequenciesHz) || any(~isfinite(phasesRad))
            error('generate_source_signal:InvalidPhases', ...
                'sourceSettings.phasesRad must be scalar or match the frequencies.');
        end

        for k = 1:numel(frequenciesHz)
            raw = raw + relativeAmplitudes(k) * ...
                sin(2 * pi * frequenciesHz(k) * tRelative + phasesRad(k));
        end
        [~, strongestIndex] = max(abs(relativeAmplitudes));
        referenceFrequencyHz = frequenciesHz(strongestIndex);
        componentFrequenciesHz = frequenciesHz;

    case {'rotor', 'rotor-like', 'harmonic'}
        fundamentalHz = settings.fundamentalHz;
        numHarmonics = settings.numHarmonics;
        rolloff = settings.harmonicRolloff;
        modulationHz = settings.modulationHz;
        modulationDepth = settings.modulationDepth;
        broadbandFraction = settings.broadbandFraction;

        local_validate_frequency(fundamentalHz, fs, ...
            'sourceSettings.fundamentalHz');
        if ~isnumeric(numHarmonics) || ~isscalar(numHarmonics) ...
                || ~isfinite(numHarmonics) || numHarmonics < 1 ...
                || numHarmonics ~= floor(numHarmonics)
            error('generate_source_signal:InvalidHarmonicCount', ...
                'sourceSettings.numHarmonics must be a positive integer.');
        end
        local_validate_nonnegative_scalar(rolloff, ...
            'sourceSettings.harmonicRolloff');
        local_validate_nonnegative_scalar(modulationHz, ...
            'sourceSettings.modulationHz');
        local_validate_fraction(modulationDepth, ...
            'sourceSettings.modulationDepth');
        local_validate_fraction(broadbandFraction, ...
            'sourceSettings.broadbandFraction');

        componentFrequenciesHz = fundamentalHz * (1:numHarmonics);
        if any(componentFrequenciesHz >= fs / 2)
            error('generate_source_signal:HarmonicAboveNyquist', ...
                ['The requested rotor harmonics reach or exceed fs/2. Reduce ' ...
                'numHarmonics or fundamentalHz, or increase fs.']);
        end
        harmonicAmplitudes = 1 ./ ((1:numHarmonics) .^ rolloff);
        phasesRad = 0.23 * (0:numHarmonics - 1);
        tonal = zeros(numSamples, 1);
        for k = 1:numHarmonics
            tonal = tonal + harmonicAmplitudes(k) * ...
                sin(2 * pi * componentFrequenciesHz(k) * tRelative + phasesRad(k));
        end
        if modulationHz > 0 && modulationDepth > 0
            amplitudeEnvelope = 1 + modulationDepth * ...
                sin(2 * pi * modulationHz * tRelative + pi / 5);
            tonal = tonal .* amplitudeEnvelope;
        end

        if broadbandFraction > 0
            broadband = local_colored_noise(numSamples, settings.noiseColor, fs);
            tonal = local_unit_rms(tonal);
            broadband = local_unit_rms(broadband);
            raw = sqrt(1 - broadbandFraction) * tonal + ...
                sqrt(broadbandFraction) * broadband;
        else
            raw = tonal;
        end
        referenceFrequencyHz = fundamentalHz;

    case {'broadband', 'noise', 'noise-burst', 'noise_burst'}
        raw = local_colored_noise(numSamples, settings.noiseColor, fs);
        envelope = local_burst_envelope(tRelative, settings.burstStartSec, ...
            settings.burstDurationSec);
        raw = raw .* envelope;

    case {'user', 'user-supplied', 'user_supplied'}
        userSignal = settings.userSignal;
        if ~isnumeric(userSignal) || ~isreal(userSignal) || ~isvector(userSignal) ...
                || isempty(userSignal) || any(~isfinite(userSignal(:)))
            error('generate_source_signal:InvalidUserSignal', ...
                'sourceSettings.userSignal must be a nonempty finite numeric vector.');
        end
        userSignal = userSignal(:);

        if ~isempty(settings.userTime)
            userTime = settings.userTime(:);
            if numel(userTime) ~= numel(userSignal) || any(~isfinite(userTime)) ...
                    || any(diff(userTime) <= 0)
                error('generate_source_signal:InvalidUserTime', ...
                    ['sourceSettings.userTime must be finite, strictly increasing, ' ...
                    'and match userSignal.']);
            end
            userTime = userTime - userTime(1);
            raw = interp1(userTime, userSignal, tRelative, 'linear', 0);
        elseif ~isempty(settings.userFs)
            local_validate_positive_scalar(settings.userFs, 'sourceSettings.userFs');
            userTime = (0:numel(userSignal) - 1).' / settings.userFs;
            raw = interp1(userTime, userSignal, tRelative, 'linear', 0);
        elseif numel(userSignal) == numSamples
            raw = userSignal;
        else
            error('generate_source_signal:UserTimeBaseRequired', ...
                ['When userSignal does not match t, provide sourceSettings.userFs ' ...
                'or sourceSettings.userTime.']);
        end

    otherwise
        error('generate_source_signal:UnknownSourceType', ...
            'Unknown source type "%s".', sourceType);
end

if settings.removeMean
    raw = raw - mean(raw);
end
sourceSignal = settings.amplitudeRms * local_unit_rms(raw);

meta = struct();
meta.type = sourceType;
meta.fs = fs;
meta.numSamples = numSamples;
meta.durationSec = t(end) - t(1);
meta.amplitudeRms = settings.amplitudeRms;
meta.referenceFrequencyHz = referenceFrequencyHz;
meta.componentFrequenciesHz = componentFrequenciesHz;
meta.actualRms = sqrt(mean(sourceSignal .^ 2));
meta.settings = local_sanitize_settings(settings);
end

function envelope = local_burst_envelope(t, startSec, durationSec)
numSamples = numel(t);
if isempty(startSec) && isempty(durationSec)
    envelope = ones(numSamples, 1);
    return;
end
if isempty(startSec)
    startSec = t(1);
end
if isempty(durationSec)
    durationSec = t(end) - startSec;
end
local_validate_nonnegative_scalar(startSec, 'sourceSettings.burstStartSec');
local_validate_positive_scalar(durationSec, 'sourceSettings.burstDurationSec');

stopSec = startSec + durationSec;
envelope = double(t >= startSec & t <= stopSec);
rampSec = min(0.05 * durationSec, 0.05);
if rampSec > 0
    rise = t >= startSec & t < startSec + rampSec;
    fall = t > stopSec - rampSec & t <= stopSec;
    envelope(rise) = 0.5 - 0.5 * cos(pi * (t(rise) - startSec) / rampSec);
    envelope(fall) = 0.5 - 0.5 * cos(pi * (stopSec - t(fall)) / rampSec);
end
end

function y = local_colored_noise(numSamples, colorName, fs)
colorName = lower(strtrim(local_to_char(colorName, 'sourceSettings.noiseColor')));
x = randn(numSamples, 1);

switch colorName
    case 'white'
        y = x;
    case 'pink'
        % A lightweight approximation formed from several one-pole bands.
        y = 0.55 * local_one_pole_lowpass(x, fs, min(250, 0.20 * fs)) + ...
            0.30 * local_one_pole_lowpass(x, fs, min(60, 0.08 * fs)) + ...
            0.15 * x;
    case {'brown', 'brownian', 'red'}
        pole = min(0.995, exp(-2 * pi * max(0.5, fs / 20000) / fs));
        y = filter(1, [1 -pole], x);
    otherwise
        error('generate_source_signal:UnknownNoiseColor', ...
            'Unknown source noise color "%s".', colorName);
end

y = local_unit_rms(y - mean(y));
end

function y = local_one_pole_lowpass(x, fs, cutoffHz)
cutoffHz = max(eps, min(cutoffHz, 0.45 * fs));
pole = exp(-2 * pi * cutoffHz / fs);
y = filter(1 - pole, [1 -pole], x);
end

function y = local_unit_rms(x)
x = x(:);
r = sqrt(mean(x .^ 2));
if r > sqrt(eps)
    y = x / r;
else
    y = zeros(size(x));
end
end

function settings = local_sanitize_settings(settings)
if isfield(settings, 'userSignal') && ~isempty(settings.userSignal)
    settings.userSignalLength = numel(settings.userSignal);
    settings.userSignal = [];
end
if isfield(settings, 'userTime') && ~isempty(settings.userTime)
    settings.userTimeLength = numel(settings.userTime);
    settings.userTime = [];
end
end

function text = local_to_char(value, name)
if isstring(value)
    if ~isscalar(value)
        error('generate_source_signal:InvalidText', '%s must be scalar text.', name);
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('generate_source_signal:InvalidText', ...
        '%s must be a character vector or scalar string.', name);
end
end

function local_validate_frequency(value, fs, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value <= 0 || value >= fs / 2
    error('generate_source_signal:InvalidFrequency', ...
        '%s must be in the open interval (0, fs/2).', name);
end
end

function local_validate_positive_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value <= 0
    error('generate_source_signal:InvalidPositiveScalar', ...
        '%s must be a positive finite numeric scalar.', name);
end
end

function local_validate_nonnegative_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value < 0
    error('generate_source_signal:InvalidNonnegativeScalar', ...
        '%s must be a nonnegative finite numeric scalar.', name);
end
end

function local_validate_finite_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('generate_source_signal:InvalidScalar', ...
        '%s must be a finite numeric scalar.', name);
end
end


function local_validate_logical_scalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) ...
        || ~isfinite(double(value)) || ~(value == 0 || value == 1)
    error('generate_source_signal:InvalidLogical', ...
        '%s must be a logical scalar.', name);
end
end

function local_validate_fraction(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value < 0 || value > 1
    error('generate_source_signal:InvalidFraction', ...
        '%s must be a finite scalar in the interval [0, 1].', name);
end
end
