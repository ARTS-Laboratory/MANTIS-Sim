function [uavXYZ, t, meta] = generate_uav_trajectory(pathType, durationSec, trajectoryRateHz, params)
%GENERATE_UAV_TRAJECTORY Create a sampled three-dimensional UAV path.
%
%   [uavXYZ, t] = GENERATE_UAV_TRAJECTORY()
%   [uavXYZ, t] = GENERATE_UAV_TRAJECTORY(pathType, durationSec, rateHz, params)
%
% Supported pathType values:
%   'straight'  - startXYZ to endXYZ
%   'arc'       - circular arc with optional altitude change
%   'loiter'    - one or more circles, optionally with vertical motion
%   'piecewise' - linear interpolation through waypoints
%   'custom'    - return params.uavXYZ and params.t after validation
%
% The trajectory sampling rate does not need to match the acoustic sampling
% rate. SIMULATE_UAV_ACOUSTICS resamples the path to its output time base.

if nargin < 1 || isempty(pathType)
    pathType = 'straight';
end
if nargin < 2 || isempty(durationSec)
    durationSec = 8;
end
if nargin < 3 || isempty(trajectoryRateHz)
    trajectoryRateHz = 20;
end
if nargin < 4 || isempty(params)
    params = struct();
end

if isstring(pathType)
    if ~isscalar(pathType)
        error('generate_uav_trajectory:InvalidPathType', ...
            'pathType must be a character vector or scalar string.');
    end
    pathType = char(pathType);
end
if ~ischar(pathType)
    error('generate_uav_trajectory:InvalidPathType', ...
        'pathType must be a character vector or scalar string.');
end
if ~isstruct(params) || ~isscalar(params)
    error('generate_uav_trajectory:InvalidParams', ...
        'params must be a scalar structure.');
end

kind = lower(strtrim(pathType));

if strcmp(kind, 'custom')
    if ~isfield(params, 'uavXYZ') || ~isfield(params, 't')
        error('generate_uav_trajectory:MissingCustomData', ...
            'Custom trajectories require params.uavXYZ and params.t.');
    end
    uavXYZ = params.uavXYZ;
    t = params.t(:);
    local_validate_trajectory(uavXYZ, t);
    meta = struct('type', 'custom', 'durationSec', t(end) - t(1), ...
        'trajectoryRateHz', 1 / median(diff(t)), 'parameters', params);
    return;
end

local_validate_positive_scalar(durationSec, 'durationSec');
local_validate_positive_scalar(trajectoryRateHz, 'trajectoryRateHz');

% linspace includes both endpoints while keeping a nearly exact requested
% sample rate for durations that are not integer multiples of 1/rate.
numSamples = max(2, round(durationSec * trajectoryRateHz) + 1);
t = linspace(0, durationSec, numSamples).';
u = t / durationSec;

switch kind
    case {'straight', 'line'}
        startXYZ = local_get(params, 'startXYZ', [-120 -50 80]);
        endXYZ = local_get(params, 'endXYZ', [120 50 80]);
        local_validate_point(startXYZ, 'params.startXYZ');
        local_validate_point(endXYZ, 'params.endXYZ');
        startXYZ = reshape(startXYZ, 1, 3);
        endXYZ = reshape(endXYZ, 1, 3);
        uavXYZ = startXYZ + u .* (endXYZ - startXYZ);

    case 'arc'
        centerXY = local_get(params, 'centerXY', [0 0]);
        radiusM = local_get(params, 'radiusM', 100);
        startAngleDeg = local_get(params, 'startAngleDeg', -90);
        endAngleDeg = local_get(params, 'endAngleDeg', 90);
        altitudeM = local_get(params, 'altitudeM', 80);
        altitudeStartM = local_get(params, 'altitudeStartM', altitudeM);
        altitudeEndM = local_get(params, 'altitudeEndM', altitudeM);

        local_validate_xy(centerXY, 'params.centerXY');
        local_validate_positive_scalar(radiusM, 'params.radiusM');
        local_validate_finite_scalar(startAngleDeg, 'params.startAngleDeg');
        local_validate_finite_scalar(endAngleDeg, 'params.endAngleDeg');
        local_validate_nonnegative_scalar(altitudeStartM, 'params.altitudeStartM');
        local_validate_nonnegative_scalar(altitudeEndM, 'params.altitudeEndM');

        theta = deg2rad(startAngleDeg + u * (endAngleDeg - startAngleDeg));
        z = altitudeStartM + u * (altitudeEndM - altitudeStartM);
        uavXYZ = [centerXY(1) + radiusM * cos(theta), ...
            centerXY(2) + radiusM * sin(theta), z];

    case {'loiter', 'circle'}
        centerXY = local_get(params, 'centerXY', [0 0]);
        radiusM = local_get(params, 'radiusM', 80);
        startAngleDeg = local_get(params, 'startAngleDeg', 0);
        revolutions = local_get(params, 'revolutions', 1);
        altitudeM = local_get(params, 'altitudeM', 75);
        altitudeVariationM = local_get(params, 'altitudeVariationM', 0);
        verticalCycles = local_get(params, 'verticalCycles', 1);

        local_validate_xy(centerXY, 'params.centerXY');
        local_validate_positive_scalar(radiusM, 'params.radiusM');
        local_validate_finite_scalar(startAngleDeg, 'params.startAngleDeg');
        local_validate_positive_scalar(revolutions, 'params.revolutions');
        local_validate_nonnegative_scalar(altitudeM, 'params.altitudeM');
        local_validate_nonnegative_scalar(altitudeVariationM, ...
            'params.altitudeVariationM');
        local_validate_positive_scalar(verticalCycles, 'params.verticalCycles');

        theta = deg2rad(startAngleDeg) + 2 * pi * revolutions * u;
        z = altitudeM + altitudeVariationM * sin(2 * pi * verticalCycles * u);
        if any(z < 0)
            error('generate_uav_trajectory:NegativeAltitude', ...
                'The loiter altitude settings produce a negative altitude.');
        end
        uavXYZ = [centerXY(1) + radiusM * cos(theta), ...
            centerXY(2) + radiusM * sin(theta), z];

    case {'piecewise', 'waypoints'}
        waypoints = local_get(params, 'waypoints', ...
            [-120 -60 70; -30 20 90; 40 -10 60; 130 70 85]);
        if ~isnumeric(waypoints) || ~isreal(waypoints) || size(waypoints, 2) ~= 3 ...
                || size(waypoints, 1) < 2 || any(~isfinite(waypoints(:)))
            error('generate_uav_trajectory:InvalidWaypoints', ...
                'params.waypoints must be a finite K-by-3 numeric array, K >= 2.');
        end
        if any(waypoints(:, 3) < 0)
            error('generate_uav_trajectory:NegativeAltitude', ...
                'Waypoint altitudes must be nonnegative.');
        end

        if isfield(params, 'waypointTimes') && ~isempty(params.waypointTimes)
            waypointTimes = params.waypointTimes(:);
            if numel(waypointTimes) ~= size(waypoints, 1) ...
                    || any(~isfinite(waypointTimes)) || any(diff(waypointTimes) <= 0)
                error('generate_uav_trajectory:InvalidWaypointTimes', ...
                    ['params.waypointTimes must be finite, strictly increasing, ' ...
                    'and match the number of waypoints.']);
            end
            durationSec = waypointTimes(end) - waypointTimes(1);
            numSamples = max(2, round(durationSec * trajectoryRateHz) + 1);
            t = linspace(waypointTimes(1), waypointTimes(end), numSamples).';
        else
            segmentLengths = sqrt(sum(diff(waypoints, 1, 1).^2, 2));
            if sum(segmentLengths) <= eps
                error('generate_uav_trajectory:CoincidentWaypoints', ...
                    'At least two waypoints must have different coordinates.');
            end
            waypointTimes = [0; cumsum(segmentLengths)] / sum(segmentLengths) * durationSec;
        end
        uavXYZ = interp1(waypointTimes, waypoints, t, 'linear');

    otherwise
        error('generate_uav_trajectory:UnknownPathType', ...
            'Unknown path type "%s".', pathType);
end

local_validate_trajectory(uavXYZ, t);
meta = struct();
meta.type = kind;
meta.durationSec = t(end) - t(1);
meta.trajectoryRateHz = 1 / median(diff(t));
meta.parameters = params;
end

function value = local_get(s, fieldName, defaultValue)
if isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function local_validate_trajectory(xyz, t)
if ~isnumeric(xyz) || ~isreal(xyz) || size(xyz, 2) ~= 3 ...
        || size(xyz, 1) ~= numel(t) || size(xyz, 1) < 2 ...
        || any(~isfinite(xyz(:)))
    error('generate_uav_trajectory:InvalidTrajectory', ...
        'The trajectory must be a finite T-by-3 numeric array matching t.');
end
if ~isnumeric(t) || ~isreal(t) || any(~isfinite(t)) || any(diff(t) <= 0)
    error('generate_uav_trajectory:InvalidTimeVector', ...
        't must be finite and strictly increasing.');
end
if any(xyz(:, 3) < 0)
    error('generate_uav_trajectory:NegativeAltitude', ...
        'UAV altitude must be nonnegative.');
end
end

function local_validate_point(value, name)
if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 ...
        || any(~isfinite(value(:)))
    error('generate_uav_trajectory:InvalidPoint', ...
        '%s must contain three finite numeric coordinates.', name);
end
if value(3) < 0
    error('generate_uav_trajectory:NegativeAltitude', ...
        '%s must have a nonnegative z coordinate.', name);
end
end

function local_validate_xy(value, name)
if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 2 ...
        || any(~isfinite(value(:)))
    error('generate_uav_trajectory:InvalidXY', ...
        '%s must contain two finite numeric coordinates.', name);
end
end

function local_validate_positive_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value <= 0
    error('generate_uav_trajectory:InvalidPositiveScalar', ...
        '%s must be a positive finite numeric scalar.', name);
end
end

function local_validate_nonnegative_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) ...
        || ~isfinite(value) || value < 0
    error('generate_uav_trajectory:InvalidNonnegativeScalar', ...
        '%s must be a nonnegative finite numeric scalar.', name);
end
end

function local_validate_finite_scalar(value, name)
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('generate_uav_trajectory:InvalidScalar', ...
        '%s must be a finite numeric scalar.', name);
end
end
