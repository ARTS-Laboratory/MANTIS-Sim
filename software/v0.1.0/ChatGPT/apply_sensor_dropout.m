function [Yout, dropoutMask, info] = apply_sensor_dropout(Y, dropoutSettings, fs)
%APPLY_SENSOR_DROPOUT Simulate missing sensors, bursts, or samples.
%
%   [Yout, mask, info] = APPLY_SENSOR_DROPOUT(Y, dropoutSettings, fs)
%
% dropoutSettings.mode:
%   'sensor' - each complete microphone channel fails with probability rate
%   'burst'  - contiguous gaps occupy approximately rate of each channel
%   'sample' - independent samples are missing with probability rate
%
% The default missing value is NaN so downstream algorithms cannot confuse a
% failed sample with a real acoustic zero.

if nargin < 2 || isempty(dropoutSettings)
    dropoutSettings = struct();
end
if nargin < 3 || isempty(fs)
    fs = 16000;
end

if ~isnumeric(Y) || ~isreal(Y) || ndims(Y) ~= 2 || isempty(Y) ...
        || any(~isfinite(Y(:)))
    error('apply_sensor_dropout:InvalidSignal', ...
        'Y must be a nonempty, finite, real numeric matrix.');
end
if ~isstruct(dropoutSettings) || ~isscalar(dropoutSettings)
    error('apply_sensor_dropout:InvalidSettings', ...
        'dropoutSettings must be a scalar structure.');
end
if ~isnumeric(fs) || ~isreal(fs) || ~isscalar(fs) || ~isfinite(fs) || fs <= 0
    error('apply_sensor_dropout:InvalidSampleRate', ...
        'fs must be a positive finite numeric scalar.');
end

allSettings = uav_acoustic_defaults(struct('dropout', dropoutSettings));
settings = allSettings.dropout;
local_validate_settings(settings);

numSamples = size(Y, 1);
numMics = size(Y, 2);
dropoutMask = false(numSamples, numMics);
mode = lower(strtrim(local_to_char(settings.mode, 'dropoutSettings.mode')));

if settings.rate > 0
    switch mode
        case {'sensor', 'channel', 'node'}
            failedChannels = rand(1, numMics) < settings.rate;
            dropoutMask(:, failedChannels) = true;

        case {'sample', 'samples'}
            dropoutMask = rand(numSamples, numMics) < settings.rate;

        case {'burst', 'bursts', 'gap'}
            durationRange = settings.burstDurationSec(:).';
            minBurstSamples = max(1, round(durationRange(1) * fs));
            maxBurstSamples = max(minBurstSamples, round(durationRange(end) * fs));
            targetCount = min(numSamples, round(settings.rate * numSamples));

            for micIndex = 1:numMics
                attempts = 0;
                maxAttempts = max(25, 10 * ceil(targetCount / minBurstSamples));
                while nnz(dropoutMask(:, micIndex)) < targetCount ...
                        && attempts < maxAttempts
                    burstLength = minBurstSamples + ...
                        randi(maxBurstSamples - minBurstSamples + 1) - 1;
                    burstLength = min(burstLength, numSamples);
                    startIndex = randi(numSamples - burstLength + 1);
                    stopIndex = startIndex + burstLength - 1;
                    dropoutMask(startIndex:stopIndex, micIndex) = true;
                    attempts = attempts + 1;
                end

                % If overlapping bursts prevented the target fraction from
                % being reached, fill the remainder from still-valid samples.
                missingCount = targetCount - nnz(dropoutMask(:, micIndex));
                if missingCount > 0
                    available = find(~dropoutMask(:, micIndex));
                    order = randperm(numel(available), min(missingCount, numel(available)));
                    dropoutMask(available(order), micIndex) = true;
                end
            end

        otherwise
            error('apply_sensor_dropout:UnknownMode', ...
                'Unknown dropout mode "%s".', mode);
    end
end

Yout = Y;
Yout(dropoutMask) = settings.value;

fractionByChannel = sum(dropoutMask, 1) / numSamples;
info = struct();
info.mode = mode;
info.requestedRate = settings.rate;
info.actualFractionByChannel = fractionByChannel;
info.failedChannels = find(fractionByChannel >= 1);
info.numDroppedSamples = nnz(dropoutMask);
info.value = settings.value;
end

function local_validate_settings(settings)
if ~isnumeric(settings.rate) || ~isreal(settings.rate) || ~isscalar(settings.rate) ...
        || ~isfinite(settings.rate) || settings.rate < 0 || settings.rate > 1
    error('apply_sensor_dropout:InvalidRate', ...
        'dropoutSettings.rate must be a finite scalar in [0, 1].');
end
if ~isnumeric(settings.value) || ~isreal(settings.value) ...
        || ~isscalar(settings.value) || isinf(settings.value)
    error('apply_sensor_dropout:InvalidValue', ...
        'dropoutSettings.value must be a real scalar that is finite or NaN.');
end
if ~isnumeric(settings.burstDurationSec) || ~isreal(settings.burstDurationSec) ...
        || isempty(settings.burstDurationSec) ...
        || any(~isfinite(settings.burstDurationSec(:))) ...
        || any(settings.burstDurationSec(:) <= 0) ...
        || numel(settings.burstDurationSec) > 2
    error('apply_sensor_dropout:InvalidBurstDuration', ...
        ['dropoutSettings.burstDurationSec must be one or two positive ' ...
        'finite values.']);
end
if numel(settings.burstDurationSec) == 2 ...
        && settings.burstDurationSec(2) < settings.burstDurationSec(1)
    error('apply_sensor_dropout:InvalidBurstDurationOrder', ...
        'The burst duration upper bound must not be smaller than the lower bound.');
end
mode = lower(strtrim(local_to_char(settings.mode, 'dropoutSettings.mode')));
validModes = {'sensor', 'channel', 'node', 'burst', 'bursts', 'gap', ...
    'sample', 'samples'};
if ~any(strcmp(mode, validModes))
    error('apply_sensor_dropout:UnknownMode', ...
        'Unknown dropout mode "%s".', mode);
end
end

function text = local_to_char(value, name)
if isstring(value)
    if ~isscalar(value)
        error('apply_sensor_dropout:InvalidText', '%s must be scalar text.', name);
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('apply_sensor_dropout:InvalidText', ...
        '%s must be a character vector or scalar string.', name);
end
end
