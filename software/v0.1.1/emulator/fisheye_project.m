function [u, v, theta, phi, inFOV, range] = fisheye_project(pWorld, cam)
%FISHEYE_PROJECT Project world points into an upward-looking fisheye image.
%
%   [u, v, theta, phi, inFOV, range] = FISHEYE_PROJECT(pWorld, cam)
%   maps N-by-3 world points to pixel coordinates for the camera model
%   cam (see MAKE_FISHEYE_CAMERA).
%
% MODEL
%   The camera sits at cam.pos looking straight up. For a point p, the
%   viewing direction is d = (p - cam.pos)/|p - cam.pos|. Writing theta
%   for the angle off the optical axis (the zenith angle) and phi for
%   the azimuth about it,
%
%       theta = acos(d_z)
%       phi   = atan2(d_y, d_x) - cam.yaw
%       r_pix = f * theta                     (equidistant lens)
%       u     = cx + r_pix*cos(phi)
%       v     = cy - r_pix*sin(phi)
%
%   Equisolid (r = 2f sin(theta/2)) and orthographic (r = f sin(theta))
%   mappings are also supported. This is the standard first-order
%   fisheye model: it captures the compression of the image toward the
%   horizon, which is the effect that matters here, and ignores the
%   residual radial distortion that a real lens calibration would add.
%
% INPUTS
%   pWorld : N-by-3 world points (meters)
%   cam    : camera struct from MAKE_FISHEYE_CAMERA
%
% OUTPUTS
%   u, v   : N-by-1 pixel coordinates (column, row), 1-based, subpixel.
%            Values are returned even when outside the FOV so that
%            partially visible objects can still be clipped sensibly.
%   theta  : N-by-1 zenith angle from the optical axis (rad)
%   phi    : N-by-1 azimuth in the camera frame (rad)
%   inFOV  : N-by-1 logical, true where theta <= cam.thetaMax
%   range  : N-by-1 distance from the camera to each point (m)
%
% See also MAKE_FISHEYE_CAMERA, RENDER_UAV_PROJECTION

n = size(pWorld, 1);
d = pWorld - repmat(cam.pos, n, 1);

range = sqrt(sum(d.^2, 2));
range = max(range, 1e-6);

dz = d(:,3) ./ range;
theta = acos(max(-1, min(1, dz)));
phi = atan2(d(:,2), d(:,1)) - cam.yaw;

switch cam.projection
    case 'equidistant',  rPix = cam.f * theta;
    case 'equisolid',    rPix = 2 * cam.f * sin(theta/2);
    case 'orthographic', rPix = cam.f * sin(min(theta, pi/2));
    otherwise
        error('fisheye_project:badProjection', ...
            'Unknown projection "%s".', cam.projection);
end

u = cam.cx + rPix .* cos(phi);
v = cam.cy - rPix .* sin(phi);

inFOV = theta <= cam.thetaMax;

end
