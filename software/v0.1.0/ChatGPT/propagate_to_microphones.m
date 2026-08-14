function [cleanY, distances, delays, doppler, info] = propagate_to_microphones( ...
    sourceSignal, t, micXY, uavXYZ, settings)
%PROPAGATE_TO_MICROPHONES Apply delay, spreading loss, Doppler, and reflection.
%
%   [cleanY, distances, delays, doppler, info] = ...
%       PROPAGATE_TO_MICROPHONES(sourceSignal, t, micXY, uavXYZ, settings)
%
% Inputs use the acoustic output time base: sourceSignal and uavXYZ must have
% one row per element of t. Microphones are assumed to be at (x, y, 0).
%
% The moving-delay model s(t-r(t)/c) already contains first-order Doppler as
% a consequence of changing path length. With settings.enableDoppler=true,
% the function additionally refines the source emission time iteratively by
% evaluating the UAV location at the retarded time. This is more accurate
% for fast motion while retaining support for arbitrary user waveforms.

if nargin < 5 || isempty(settings)
    settings = struct();
end
settings = uav_acoustic_defaults(settings);

[sourceSignal, t, micXY, uavXYZ] = local_validate_inputs( ...
    sourceSignal, t, micXY, uavXYZ);

numSamples = numel(t);
numMics = size(micXY, 1);
dt = median(diff(t));
fs = 1 / dt;

local_validate_settings(settings, numMics);
interpMethod = lower(strtrim(local_to_char(settings.sourceInterpolation, ...
    'settings.sourceInterpolation')));

cleanY = zeros(numSamples, numMics);
distances = zeros(numSamples, numMics);
delays = zeros(numSamples, numMics);
radialVelocity = zeros(numSamples, numMics);
dopplerRatio = zeros(numSamples, numMics);
shiftHz = NaN(numSamples, numMics);

referenceFrequencyHz = local_reference_frequency(settings.source);
if isfield(settings, 'sourceReferenceFrequencyHz') ...
        && ~isempty(settings.sourceReferenceFrequencyHz)
    referenceFrequencyHz = settings.sourceReferenceFrequencyHz;
end

reflectionExponent = settings.reflection.attenuationExponent;
if isempty(reflectionExponent)
    reflectionExponent = settings.alpha;
end

minDirectGain = inf(1, numMics);
maxDirectGain = zeros(1, numMics);
clippedDopplerSamples = zeros(1, numMics);

for micIndex = 1:numMics
    mic = [micXY(micIndex, :) 0];
    delta = uavXYZ - mic;
    receiveDistance = sqrt(sum(delta .^ 2, 2));
    receiveDelay = receiveDistance / settings.c;

    distances(:, micIndex) = receiveDistance;
    delays(:, micIndex) = receiveDelay;

    % Estimate radial velocity and a conventional source-motion Doppler ratio.
    vRadial = gradient(receiveDistance, dt);
    ratioUnclipped = settings.c ./ (settings.c + vRadial);
    ratio = min(max(ratioUnclipped, settings.minDopplerRatio), ...
        settings.maxDopplerRatio);
    clippedDopplerSamples(micIndex) = nnz(ratio ~= ratioUnclipped);
    radialVelocity(:, micIndex) = vRadial;
    dopplerRatio(:, micIndex) = ratio;
    if isfinite(referenceFrequencyHz)
        shiftHz(:, micIndex) = referenceFrequencyHz * (ratio - 1);
    end

    % Receiver-time distance is the inexpensive baseline. Retarded-time
    % iterations account for where the moving source was when sound left it.
    emissionDistance = receiveDistance;
    emissionTime = t - emissionDistance / settings.c;
    if settings.enableDoppler
        for iteration = 1:settings.retardedTimeIterations
            sourcePosition = local_position_at_time(t, uavXYZ, emissionTime);
            emissionDelta = sourcePosition - mic;
            emissionDistance = sqrt(sum(emissionDelta .^ 2, 2));
            emissionTime = t - emissionDistance / settings.c;
        end
    end

    directSignal = interp1(t, sourceSignal, emissionTime, ...
        interpMethod, 0);
    directGain = (settings.referenceDistanceM ./ ...
        max(emissionDistance, settings.minDistanceM)) .^ settings.alpha;
    channel = directGain .* directSignal;

    minDirectGain(micIndex) = min(directGain);
    maxDirectGain(micIndex) = max(directGain);

    if settings.enableReflection && settings.reflection.coefficient ~= 0
        % Image source across a ground plane located groundOffsetM below the
        % microphone coordinate plane. For groundOffsetM > 0, this creates a
        % distinct, longer second path while microphones remain at z = 0.
        reflectionTime = t - local_reflection_distance( ...
            uavXYZ, micXY(micIndex, :), settings.reflection.groundOffsetM) ...
            / settings.c;
        reflectionDistance = local_reflection_distance( ...
            uavXYZ, micXY(micIndex, :), settings.reflection.groundOffsetM);

        if settings.enableDoppler
            for iteration = 1:settings.retardedTimeIterations
                sourcePosition = local_position_at_time(t, uavXYZ, reflectionTime);
                reflectionDistance = local_reflection_distance( ...
                    sourcePosition, micXY(micIndex, :), ...
                    settings.reflection.groundOffsetM);
                reflectionTime = t - reflectionDistance / settings.c;
            end
        end

        reflectedSignal = interp1(t, sourceSignal, reflectionTime, ...
            interpMethod, 0);
        reflectionGain = settings.reflection.coefficient * ...
            (settings.referenceDistanceM ./ ...
            max(reflectionDistance, settings.minDistanceM)) .^ reflectionExponent;
        channel = channel + reflectionGain .* reflectedSignal;
    end

    cleanY(:, micIndex) = channel;
end

doppler = struct();
doppler.radialVelocityMps = radialVelocity;
doppler.ratio = dopplerRatio;
doppler.shiftHz = shiftHz;
doppler.referenceFrequencyHz = referenceFrequencyHz;
doppler.enableDoppler = logical(settings.enableDoppler);
doppler.model = local_doppler_model_name(settings.enableDoppler);

info = struct();
info.fs = fs;
info.numSamples = numSamples;
info.numMics = numMics;
info.distanceMinM = min(distances, [], 1);
info.distanceMaxM = max(distances, [], 1);
info.delayMinSec = min(delays, [], 1);
info.delayMaxSec = max(delays, [], 1);
info.directGainMin = minDirectGain;
info.directGainMax = maxDirectGain;
info.referenceDistanceM = settings.referenceDistanceM;
info.attenuationExponent = settings.alpha;
info.reflectionEnabled = logical(settings.enableReflection);
info.reflectionCoefficient = settings.reflection.coefficient;
info.reflectionGroundOffsetM = settings.reflection.groundOffsetM;
info.retardedTimeIterations = settings.retardedTimeIterations;
info.dopplerClippedSamples = clippedDopplerSamples;
end

function [sourceSignal, t, micXY, uavXYZ] = local_validate_inputs( ...
    sourceSignal, t, micXY, uavXYZ)
if ~isnumeric(t) || ~isreal(t) || ~isvector(t) || numel(t) < 2 ...
        || any(~isfinite(t(:))) || any(diff(t(:)) <= 0)
    error('propagate_to_microphones:InvalidTimeVector', ...
        't must be a finite, strictly increasing numeric vector.');
end
t = t(:);

if ~isnumeric(sourceSignal) || ~isreal(sourceSignal) ...
        || ~isvector(sourceSignal) || numel(sourceSignal) ~= numel(t) ...
        || any(~isfinite(sourceSignal(:)))
    error('propagate_to_microphones:InvalidSourceSignal', ...
        'sourceSignal must be a finite numeric vector matching t.');
end
sourceSignal = sourceSignal(:);

if ~isnumeric(micXY) || ~isreal(micXY) || size(micXY, 2) ~= 2 ...
        || isempty(micXY) || any(~isfinite(micXY(:)))
    error('propagate_to_microphones:InvalidMicrophones', ...
        'micXY must be a nonempty finite N-by-2 numeric array.');
end

if ~isnumeric(uavXYZ) || ~isreal(uavXYZ) || size(uavXYZ, 2) ~= 3 ...
        || size(uavXYZ, 1) ~= numel(t) || any(~isfinite(uavXYZ(:)))
    error('propagate_to_microphones:InvalidTrajectory', ...
        'uavXYZ must be a finite T-by-3 array matching t.');
end
if any(uavXYZ(:, 3) < 0)
    error('propagate_to_microphones:NegativeAltitude', ...
        'UAV altitude must be nonnegative.');
end
end

function local_validate_settings(settings, numMics)
local_validate_positive_scalar(settings.c, 'settings.c');
local_validate_positive_scalar(settings.alpha, 'settings.alpha');
local_validate_positive_scalar(settings.referenceDistanceM, ...
    'settings.referenceDistanceM');
local_validate_positive_scalar(settings.minDistanceM, 'settings.minDistanceM');
local_validate_logical_scalar(settings.enableDoppler, 'settings.enableDoppler');
local_validate_logical_scalar(settings.enableReflection, 'settings.enableReflection');

if ~isnumeric(settings.retardedTimeIterations) ...
        || ~isscalar(settings.retardedTimeIterations) ...
        || ~isfinite(settings.retardedTimeIterations) ...
        || settings.retardedTimeIterations < 0 ...
        || settings.retardedTimeIterations ~= floor(settings.retardedTimeIterations)
    error('propagate_to_microphones:InvalidIterations', ...
        'settings.retardedTimeIterations must be a nonnegative integer.');
end
local_validate_positive_scalar(settings.minDopplerRatio, ...
    'settings.minDopplerRatio');
local_validate_positive_scalar(settings.maxDopplerRatio, ...
    'settings.maxDopplerRatio');
if settings.maxDopplerRatio < settings.minDopplerRatio
    error('propagate_to_microphones:InvalidDopplerLimits', ...
        'maxDopplerRatio must be greater than or equal to minDopplerRatio.');
end

if ~isnumeric(settings.reflection.coefficient) ...
        || ~isscalar(settings.reflection.coefficient) ...
        || ~isreal(settings.reflection.coefficient) ...
        || ~isfinite(settings.reflection.coefficient) ...
        || abs(settings.reflection.coefficient) > 1
    error('propagate_to_microphones:InvalidReflectionCoefficient', ...
        'settings.reflection.coefficient must be a real scalar in [-1, 1].');
end
local_validate_nonnegative_scalar(settings.reflection.groundOffsetM, ...
    'settings.reflection.groundOffsetM');
if ~isempty(settings.reflection.attenuationExponent)
    local_validate_positive_scalar(settings.reflection.attenuationExponent, ...
        'settings.reflection.attenuationExponent');
end

method = local_to_char(settings.sourceInterpolation, ...
    'settings.sourceInterpolation');
method = lower(strtrim(method));
validMethods = {'linear', 'nearest', 'pchip', 'spline', 'makima'};
if ~ischar(method) || ~any(strcmpi(method, validMethods))
    error('propagate_to_microphones:InvalidInterpolationMethod', ...
        'sourceInterpolation must be linear, nearest, pchip, spline, or makima.');
end

if numMics < 1
    error('propagate_to_microphones:NoMicrophones', ...
        'At least one microphone is required.');
end
end

function position = local_position_at_time(t, uavXYZ, queryTime)
% Clamp to the supplied trajectory interval. Source samples queried before
% t(1) are zero, so endpoint geometry is sufficient for those iterations.
queryClamped = min(max(queryTime, t(1)), t(end));
position = interp1(t, uavXYZ, queryClamped, 'linear');
end

function distance = local_reflection_distance(uavXYZ, micXY, groundOffsetM)
dx = uavXYZ(:, 1) - micXY(1);
dy = uavXYZ(:, 2) - micXY(2);
% The image-source vertical separation from a microphone at z=0 is
% z_uav + 2*groundOffsetM.
dzImage = uavXYZ(:, 3) + 2 * groundOffsetM;
distance = sqrt(dx .^ 2 + dy .^ 2 + dzImage .^ 2);
end

function referenceFrequencyHz = local_reference_frequency(sourceSettings)
referenceFrequencyHz = NaN;
if ~isstruct(sourceSettings) || ~isfield(sourceSettings, 'type')
    return;
end
sourceType = sourceSettings.type;
if isstring(sourceType)
    sourceType = char(sourceType);
end
if ~ischar(sourceType)
    return;
end
switch lower(strtrim(sourceType))
    case {'tone', 'single-tone', 'single_tone', 'tone-noise', ...
            'tone+noise', 'tone-plus-noise', 'tone_noise'}
        referenceFrequencyHz = sourceSettings.frequencyHz;
    case {'rotor', 'rotor-like', 'harmonic'}
        referenceFrequencyHz = sourceSettings.fundamentalHz;
    case {'multitone', 'multi-tone', 'multi_tone'}
        if ~isempty(sourceSettings.frequenciesHz)
            amplitudes = sourceSettings.relativeAmplitudes;
            if numel(amplitudes) == numel(sourceSettings.frequenciesHz)
                [~, index] = max(abs(amplitudes));
            else
                index = 1;
            end
            referenceFrequencyHz = sourceSettings.frequenciesHz(index);
        end
end
if ~isnumeric(referenceFrequencyHz) || ~isscalar(referenceFrequencyHz) ...
        || ~isfinite(referenceFrequencyHz)
    referenceFrequencyHz = NaN;
end
end

function name = local_doppler_model_name(enabled)
if enabled
    name = 'iterative retarded-time moving delay';
else
    name = 'receiver-time moving delay';
end
end


function text = local_to_char(value, name)
if isstring(value)
    if ~isscalar(value)
        error('propagate_to_microphones:InvalidText', ...
            '%s must be scalar text.', name);
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('propagate_to_microphones:InvalidText', ...
        '%s must be a character vector or scalar string.', name);
end
end

function local_validate_positive_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value <= 0
    error('propagate_to_microphones:InvalidPositiveScalar', ...
        '%s must be a positive finite numeric scalar.', name);
end
end

function local_validate_nonnegative_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value < 0
    error('propagate_to_microphones:InvalidNonnegativeScalar', ...
        '%s must be a nonnegative finite numeric scalar.', name);
end
end

function local_validate_logical_scalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) ...
        || ~isfinite(double(value)) || ~(value == 0 || value == 1)
    error('propagate_to_microphones:InvalidLogical', ...
        '%s must be a logical scalar.', name);
end
end
