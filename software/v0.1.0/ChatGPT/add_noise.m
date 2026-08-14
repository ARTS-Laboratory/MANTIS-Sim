function [Y, noise, info] = add_noise(cleanY, snrDb, noiseSettings, fs)
%ADD_NOISE Add independent and optionally correlated ambient noise.
%
%   [Y, noise, info] = ADD_NOISE(cleanY, snrDb, noiseSettings, fs)
%
% cleanY is numSamples-by-numMics. snrDb is a scalar target SNR measured
% separately for each channel. Use Inf for noise-free output. Supported
% noiseSettings.type values are 'white', 'pink', 'brown', 'wind', and
% 'mixed'. No Signal Processing Toolbox functions are required.

if nargin < 2 || isempty(snrDb)
    snrDb = 10;
end
if nargin < 3 || isempty(noiseSettings)
    noiseSettings = struct();
end
if nargin < 4 || isempty(fs)
    fs = 16000;
end

if ~isnumeric(cleanY) || ~isreal(cleanY) || ndims(cleanY) ~= 2 ...
        || isempty(cleanY) || any(~isfinite(cleanY(:)))
    error('add_noise:InvalidCleanSignal', ...
        'cleanY must be a nonempty, finite, real numeric matrix.');
end
if ~isnumeric(snrDb) || ~isreal(snrDb) || ~isscalar(snrDb) ...
        || isnan(snrDb) || snrDb == -Inf
    error('add_noise:InvalidSNR', ...
        'snrDb must be a finite numeric scalar or positive Inf.');
end
if ~isnumeric(fs) || ~isreal(fs) || ~isscalar(fs) || ~isfinite(fs) || fs <= 0
    error('add_noise:InvalidSampleRate', ...
        'fs must be a positive finite numeric scalar.');
end
if ~isstruct(noiseSettings) || ~isscalar(noiseSettings)
    error('add_noise:InvalidSettings', ...
        'noiseSettings must be a scalar structure.');
end

allSettings = uav_acoustic_defaults(struct('noise', noiseSettings));
settings = allSettings.noise;
local_validate_noise_settings(settings, fs);

numSamples = size(cleanY, 1);
numMics = size(cleanY, 2);
noise = zeros(numSamples, numMics);

noiseType = lower(strtrim(local_to_char(settings.type, 'noiseSettings.type')));
commonBase = local_make_noise(numSamples, noiseType, fs, settings);
commonBase = local_unit_rms(commonBase);

signalRms = sqrt(mean(cleanY .^ 2, 1));
targetNoiseRms = zeros(1, numMics);
actualNoiseRms = zeros(1, numMics);
actualSnrDb = inf(1, numMics);

for micIndex = 1:numMics
    independentBase = local_make_noise(numSamples, noiseType, fs, settings);
    base = sqrt(settings.commonFraction) * commonBase + ...
        sqrt(1 - settings.commonFraction) * local_unit_rms(independentBase);

    if settings.windFraction > 0 && ~strcmp(noiseType, 'wind')
        wind = local_wind_noise(numSamples, fs, settings.windCutoffHz);
        base = sqrt(1 - settings.windFraction) * local_unit_rms(base) + ...
            sqrt(settings.windFraction) * local_unit_rms(wind);
    end
    if settings.removeMean
        base = base - mean(base);
    end
    base = local_unit_rms(base);

    if isinf(snrDb)
        target = 0;
    else
        target = signalRms(micIndex) / (10 ^ (snrDb / 20));
    end
    if ~isempty(settings.ambientRms)
        target = max(target, settings.ambientRms);
    end

    noise(:, micIndex) = target * base;
    targetNoiseRms(micIndex) = target;
    actualNoiseRms(micIndex) = sqrt(mean(noise(:, micIndex) .^ 2));

    if actualNoiseRms(micIndex) > 0 && signalRms(micIndex) > 0
        actualSnrDb(micIndex) = 20 * log10( ...
            signalRms(micIndex) / actualNoiseRms(micIndex));
    elseif actualNoiseRms(micIndex) > 0
        actualSnrDb(micIndex) = -Inf;
    end
end

Y = cleanY + noise;

info = struct();
info.type = noiseType;
info.requestedSnrDb = snrDb;
info.signalRms = signalRms;
info.targetNoiseRms = targetNoiseRms;
info.actualNoiseRms = actualNoiseRms;
info.actualSnrDb = actualSnrDb;
info.commonFraction = settings.commonFraction;
info.windFraction = settings.windFraction;
info.windCutoffHz = settings.windCutoffHz;
info.ambientRms = settings.ambientRms;
end

function y = local_make_noise(numSamples, noiseType, fs, settings)
x = randn(numSamples, 1);
switch noiseType
    case 'white'
        y = x;
    case 'pink'
        y = 0.55 * local_one_pole_lowpass(x, fs, min(250, 0.20 * fs)) + ...
            0.30 * local_one_pole_lowpass(x, fs, min(60, 0.08 * fs)) + ...
            0.15 * x;
    case {'brown', 'brownian', 'red'}
        pole = 0.995;
        y = filter(1, [1 -pole], x);
    case 'wind'
        y = local_wind_noise(numSamples, fs, settings.windCutoffHz);
    case 'mixed'
        pink = 0.55 * local_one_pole_lowpass(x, fs, min(250, 0.20 * fs)) + ...
            0.30 * local_one_pole_lowpass(x, fs, min(60, 0.08 * fs)) + ...
            0.15 * x;
        wind = local_wind_noise(numSamples, fs, settings.windCutoffHz);
        y = 0.65 * x + 0.25 * local_unit_rms(pink) + ...
            0.10 * local_unit_rms(wind);
    otherwise
        error('add_noise:UnknownNoiseType', ...
            'Unknown noise type "%s".', noiseType);
end
end

function y = local_wind_noise(numSamples, fs, cutoffHz)
x = randn(numSamples, 1);
y = local_one_pole_lowpass(x, fs, cutoffHz);
% A second pole makes the low-frequency component smoother and more gust-like.
y = local_one_pole_lowpass(y, fs, min(0.75 * cutoffHz, 0.45 * fs));
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

function local_validate_noise_settings(settings, fs)
local_validate_fraction(settings.commonFraction, ...
    'noiseSettings.commonFraction');
local_validate_fraction(settings.windFraction, ...
    'noiseSettings.windFraction');
local_validate_logical_scalar(settings.removeMean, ...
    'noiseSettings.removeMean');
if ~isempty(settings.ambientRms)
    if ~isnumeric(settings.ambientRms) || ~isreal(settings.ambientRms) ...
            || ~isscalar(settings.ambientRms) || ~isfinite(settings.ambientRms) ...
            || settings.ambientRms < 0
        error('add_noise:InvalidAmbientRms', ...
            'noiseSettings.ambientRms must be empty or a nonnegative scalar.');
    end
end
if ~isnumeric(settings.windCutoffHz) || ~isreal(settings.windCutoffHz) ...
        || ~isscalar(settings.windCutoffHz) || ~isfinite(settings.windCutoffHz) ...
        || settings.windCutoffHz <= 0 || settings.windCutoffHz >= fs / 2
    error('add_noise:InvalidWindCutoff', ...
        'noiseSettings.windCutoffHz must be in the open interval (0, fs/2).');
end
local_to_char(settings.type, 'noiseSettings.type');
end

function text = local_to_char(value, name)
if isstring(value)
    if ~isscalar(value)
        error('add_noise:InvalidText', '%s must be scalar text.', name);
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('add_noise:InvalidText', ...
        '%s must be a character vector or scalar string.', name);
end
end


function local_validate_logical_scalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) ...
        || ~isfinite(double(value)) || ~(value == 0 || value == 1)
    error('add_noise:InvalidLogical', ...
        '%s must be a logical scalar.', name);
end
end

function local_validate_fraction(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value < 0 || value > 1
    error('add_noise:InvalidFraction', ...
        '%s must be a finite scalar in the interval [0, 1].', name);
end
end
