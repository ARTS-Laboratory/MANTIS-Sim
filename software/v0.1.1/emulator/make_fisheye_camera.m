function cam = make_fisheye_camera(pos, varargin)
%MAKE_FISHEYE_CAMERA Build an upward-looking fisheye camera model.
%
%   cam = MAKE_FISHEYE_CAMERA(pos) creates a camera sitting at world
%   position pos = [x y z] with its optical axis pointing straight up
%   (+z), matching the ground-camera packages used for under-structure
%   UAV tracking.
%
% NAME-VALUE OPTIONS
%   'imageSize'   [rows cols] in pixels (default: [240 240])
%   'fovDeg'      full field of view across the image circle
%                 (default: 185, typical of a circular fisheye)
%   'yawDeg'      rotation of the camera about the vertical axis, i.e.
%                 which world direction appears "up" in the image
%                 (default: 0, world +y toward the top of the frame)
%   'projection'  'equidistant' (default, r = f*theta), 'equisolid'
%                 (r = 2f*sin(theta/2)), or 'orthographic'
%                 (r = f*sin(theta))
%   'vignetteExp' exponent of the cos(theta) illumination falloff
%                 (default: 0.7), floored at 0.30. Real circular
%                 fisheyes fall off far more gently than the cos^4 law
%                 that applies to rectilinear lenses.
%
% OUTPUT FIELDS
%   .pos, .yaw, .imageSize, .fovDeg, .thetaMax, .projection
%   .f, .cx, .cy         intrinsics in pixels
%   .thetaMap, .phiMap   per-pixel zenith and azimuth angle maps (rad)
%   .circleMask          logical map of pixels inside the image circle
%   .vignette            per-pixel illumination factor (includes mask)
%   .pixPerRad           angular resolution, useful for range budgeting
%
% ANGULAR RESOLUTION NOTE
%   With the defaults (240 px across a 185 deg FOV) one pixel subtends
%   about 0.77 deg (13.5 mrad). A 2.6 m wingspan therefore shrinks to
%   roughly one pixel at about 190 m slant range, and a 0.8 m
%   multirotor at about 60 m. Increase 'imageSize' or narrow 'fovDeg'
%   to buy detection range.
%
% See also FISHEYE_PROJECT, SIMULATE_UAV_CAMERAS

p = inputParser;
p.FunctionName = 'make_fisheye_camera';
addRequired(p, 'pos');
addParameter(p, 'imageSize', [240 240]);
addParameter(p, 'fovDeg', 185);
addParameter(p, 'yawDeg', 0);
addParameter(p, 'projection', 'equidistant');
addParameter(p, 'vignetteExp', 0.7);
parse(p, pos, varargin{:});
opt = p.Results;

pos = pos(:)';
if numel(pos) ~= 3
    error('make_fisheye_camera:badPos', 'pos must be a 3-element [x y z].');
end

H = opt.imageSize(1);
W = opt.imageSize(2);
thetaMax = (opt.fovDeg/2) * pi/180;
R = min(H, W)/2;                 % image-circle radius in pixels

cam = struct();
cam.pos        = pos;
cam.yaw        = opt.yawDeg * pi/180;
cam.imageSize  = [H W];
cam.fovDeg     = opt.fovDeg;
cam.thetaMax   = thetaMax;
cam.projection = lower(opt.projection);
cam.cx         = (W + 1)/2;
cam.cy         = (H + 1)/2;
cam.imageRadius = R;

% Focal length in pixels, chosen so the FOV edge lands on the image
% circle for the selected projection.
switch cam.projection
    case 'equidistant',   cam.f = R / thetaMax;
    case 'equisolid',     cam.f = R / (2*sin(thetaMax/2));
    case 'orthographic',  cam.f = R / sin(min(thetaMax, pi/2));
    otherwise
        error('make_fisheye_camera:badProjection', ...
            'projection must be equidistant, equisolid, or orthographic.');
end
cam.pixPerRad = cam.f;

% --- per-pixel angle maps --------------------------------------------
[uu, vv] = meshgrid(1:W, 1:H);
du = uu - cam.cx;
dv = cam.cy - vv;                % image v grows downward
rPix = sqrt(du.^2 + dv.^2);

switch cam.projection
    case 'equidistant',  theta = rPix / cam.f;
    case 'equisolid',    theta = 2*asin(min(1, rPix./(2*cam.f)));
    case 'orthographic', theta = asin(min(1, rPix./cam.f));
end
phi = atan2(dv, du) + cam.yaw;   % azimuth in world frame

cam.thetaMap   = theta;
cam.phiMap     = phi;
cam.circleMask = (rPix <= R) & (theta <= thetaMax);

vig = cos(min(theta, pi/2)) .^ opt.vignetteExp;
vig = max(vig, 0.30);
cam.vignette = vig .* double(cam.circleMask);

end
