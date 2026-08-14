function [uavXYZ, t] = generate_uav_trajectory(t, pathType, varargin)
%GENERATE_UAV_TRAJECTORY Create a 3D UAV trajectory for simulation.
%
%   [uavXYZ, t] = GENERATE_UAV_TRAJECTORY(t, pathType, ...) builds a
%   T-by-3 array of UAV (x,y,z) positions sampled at the times in t.
%
% INPUTS
%   t        : time vector in seconds (used to determine number of
%              samples and total duration).
%   pathType : 'straight' | 'arc' | 'loiter' | 'piecewise'
%
% NAME-VALUE OPTIONS (apply per pathType)
%   Common:
%     'altitude'   constant flight altitude in meters (default: 80)
%
%   'straight':
%     'startXY'    [x y] start point (default: [-200 0])
%     'endXY'      [x y] end point   (default: [ 200 0])
%
%   'arc':
%     'center'     [x y] center of the arc (default: [0 0])
%     'radius'     radius in meters (default: 150)
%     'startAngle' start angle in radians (default: 0)
%     'endAngle'   end angle in radians (default: pi)
%
%   'loiter':
%     'center'     [x y] center of the circular loiter (default: [0 0])
%     'radius'     radius in meters (default: 100)
%     'numLoops'   number of full revolutions over the duration (default: 2)
%
%   'piecewise':
%     'waypoints'  M-by-3 array of (x,y,z) waypoints. The UAV moves
%                  through these waypoints at constant speed, spending
%                  time proportional to segment length. (required)
%
% OUTPUT
%   uavXYZ : T-by-3 array of UAV positions, T = numel(t)
%   t      : the same time vector passed in (returned for convenience)
%
% EXAMPLE
%   t = (0:1/2000:10)';
%   uavXYZ = generate_uav_trajectory(t, 'arc', 'radius', 150, ...
%       'startAngle', 0, 'endAngle', pi, 'altitude', 100);

p = inputParser;
p.FunctionName = 'generate_uav_trajectory';
addRequired(p, 't');
addRequired(p, 'pathType');
addParameter(p, 'altitude', 80);
addParameter(p, 'startXY', [-200 0]);
addParameter(p, 'endXY', [200 0]);
addParameter(p, 'center', [0 0]);
addParameter(p, 'radius', 150);
addParameter(p, 'startAngle', 0);
addParameter(p, 'endAngle', pi);
addParameter(p, 'numLoops', 2);
addParameter(p, 'waypoints', []);
parse(p, t, pathType, varargin{:});
opt = p.Results;

t = t(:);
T = numel(t);
if T < 2
    error('generate_uav_trajectory:tooFewSamples', 't must have at least 2 samples.');
end

duration = t(end) - t(1);
frac = (t - t(1)) / duration; % 0..1 normalized progress

switch lower(pathType)
    case 'straight'
        x = opt.startXY(1) + frac * (opt.endXY(1) - opt.startXY(1));
        y = opt.startXY(2) + frac * (opt.endXY(2) - opt.startXY(2));
        z = opt.altitude * ones(T, 1);

    case 'arc'
        theta = opt.startAngle + frac * (opt.endAngle - opt.startAngle);
        x = opt.center(1) + opt.radius * cos(theta);
        y = opt.center(2) + opt.radius * sin(theta);
        z = opt.altitude * ones(T, 1);

    case 'loiter'
        theta = 2*pi*opt.numLoops * frac;
        x = opt.center(1) + opt.radius * cos(theta);
        y = opt.center(2) + opt.radius * sin(theta);
        z = opt.altitude * ones(T, 1);

    case 'piecewise'
        wp = opt.waypoints;
        if isempty(wp) || size(wp, 2) ~= 3 || size(wp, 1) < 2
            error('generate_uav_trajectory:badWaypoints', ...
                'waypoints must be an M-by-3 array with at least 2 rows.');
        end
        % Cumulative arc-length parametrization -> constant speed path.
        segLen = sqrt(sum(diff(wp).^2, 2));
        cumLen = [0; cumsum(segLen)];
        totalLen = cumLen(end);
        if totalLen <= 0
            error('generate_uav_trajectory:degenerateWaypoints', ...
                'Waypoints must not all coincide.');
        end
        targetLen = frac * totalLen;
        x = interp1(cumLen, wp(:,1), targetLen, 'linear');
        y = interp1(cumLen, wp(:,2), targetLen, 'linear');
        z = interp1(cumLen, wp(:,3), targetLen, 'linear');

    otherwise
        error('generate_uav_trajectory:badPathType', ...
            'Unknown pathType "%s". Use straight, arc, loiter, or piecewise.', pathType);
end

uavXYZ = [x(:), y(:), z(:)];

end
