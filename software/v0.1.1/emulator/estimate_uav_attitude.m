function att = estimate_uav_attitude(uavXYZ, t, varargin)
%ESTIMATE_UAV_ATTITUDE Infer UAV attitude from its trajectory.
%
%   att = ESTIMATE_UAV_ATTITUDE(uavXYZ, t) returns a T-by-3 array of
%   [yaw pitch roll] in radians, derived from the flight path. This is a
%   kinematic estimate, not a 6-DOF flight dynamics model: the airframe
%   is assumed to fly along its velocity vector and to bank as needed to
%   turn. It exists so the rendered silhouette is oriented sensibly.
%
% NAME-VALUE OPTIONS
%   'mode'        'fixedwing' (default) | 'multirotor'
%                 fixedwing : nose along velocity, coordinated-turn bank
%                 multirotor: nose along velocity (or held), small pitch
%                             and roll proportional to acceleration
%   'maxBankDeg'  bank limit in degrees (default: 35 fixed wing, 25 multi)
%   'smoothSec'   moving-average window applied to the angles, in seconds
%                 (default: 0.5). Suppresses differentiation noise.
%   'fixedYawDeg' if supplied, yaw is held at this constant heading
%                 instead of following the path (useful for a hovering
%                 multirotor pointed at a target).
%
% OUTPUT
%   att : T-by-3 [yaw pitch roll] in radians.
%         yaw   = heading, CCW from the world +x axis
%         pitch = nose up positive
%         roll  = right wing down positive
%
% See also UAV_PLATFORM, SIMULATE_UAV_CAMERAS

p = inputParser;
p.FunctionName = 'estimate_uav_attitude';
addParameter(p, 'mode', 'fixedwing');
addParameter(p, 'maxBankDeg', []);
addParameter(p, 'smoothSec', 0.5);
addParameter(p, 'fixedYawDeg', []);
parse(p, varargin{:});
opt = p.Results;

t = t(:);
T = numel(t);
if size(uavXYZ, 1) ~= T
    error('estimate_uav_attitude:sizeMismatch', ...
        'uavXYZ must have the same number of rows as numel(t).');
end

isFixedWing = strcmpi(opt.mode, 'fixedwing');
if isempty(opt.maxBankDeg)
    if isFixedWing, opt.maxBankDeg = 35; else, opt.maxBankDeg = 25; end
end
maxBank = opt.maxBankDeg * pi/180;
gAccel = 9.81;

% --- velocity ---------------------------------------------------------
vx = gradient(uavXYZ(:,1), t);
vy = gradient(uavXYZ(:,2), t);
vz = gradient(uavXYZ(:,3), t);
vGround = sqrt(vx.^2 + vy.^2);
speed = sqrt(vx.^2 + vy.^2 + vz.^2);

% --- yaw --------------------------------------------------------------
if ~isempty(opt.fixedYawDeg)
    yaw = opt.fixedYawDeg * pi/180 * ones(T, 1);
else
    yaw = atan2(vy, vx);
    % Hold the last valid heading wherever the vehicle is essentially
    % stationary (hover), where atan2 of numerical noise is meaningless.
    moving = vGround > 1e-3;
    if ~any(moving)
        yaw = zeros(T, 1);
    else
        firstMove = find(moving, 1, 'first');
        yaw(1:firstMove) = yaw(firstMove);
        for k = 2:T
            if ~moving(k), yaw(k) = yaw(k-1); end
        end
    end
    yaw = unwrap(yaw);
end

% --- pitch ------------------------------------------------------------
pitch = atan2(vz, max(vGround, 1e-6));

% --- roll -------------------------------------------------------------
yawRate = gradient(yaw, t);
if isFixedWing
    % Coordinated turn: tan(phi) = V * psi_dot / g
    roll = atan2(speed .* yawRate, gAccel);
else
    % Multirotor: bank into the lateral acceleration, and the "pitch"
    % of the airframe leans into the forward acceleration.
    ax = gradient(vx, t);
    ay = gradient(vy, t);
    aLat = -ax.*sin(yaw) + ay.*cos(yaw);
    aFwd =  ax.*cos(yaw) + ay.*sin(yaw);
    roll  = atan2(aLat, gAccel);
    pitch = pitch + atan2(aFwd, gAccel);
end
roll = max(min(roll, maxBank), -maxBank);

% --- smoothing --------------------------------------------------------
if opt.smoothSec > 0 && T > 4
    dt = median(diff(t));
    win = max(3, round(opt.smoothSec / dt));
    yaw   = moving_average(yaw,   win);
    pitch = moving_average(pitch, win);
    roll  = moving_average(roll,  win);
end

att = [yaw, pitch, roll];

end

% =====================================================================
function y = moving_average(x, win)
%MOVING_AVERAGE Centered moving average with edge-value padding.
%   Implemented with conv so no toolbox or recent-release function is
%   required (works in older MATLAB and in Octave).
if mod(win, 2) == 0, win = win + 1; end
half = (win - 1) / 2;
xp = [repmat(x(1), half, 1); x(:); repmat(x(end), half, 1)];
k = ones(win, 1) / win;
yFull = conv(xp, k, 'same');
y = yFull(half + (1:numel(x)));
end
