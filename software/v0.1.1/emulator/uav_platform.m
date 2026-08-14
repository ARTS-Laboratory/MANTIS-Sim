function plat = uav_platform(name, varargin)
%UAV_PLATFORM Geometry, appearance, and suggested acoustics for a UAV type.
%
%   plat = UAV_PLATFORM(name) returns a struct describing a UAV as a small
%   set of flat polygons in the BODY frame, together with nav-light
%   positions and suggested acoustic source settings. The polygons are
%   what the ground cameras "see": they are projected through the fisheye
%   lens and rasterized by RENDER_UAV_PROJECTION.
%
% SUPPORTED NAMES
%   'swift'  | 'swift_uav' | 'fixedwing'  -> SWIFT-UAV, the 20 kg-class
%       open-source fixed-wing research platform (2.6 m span, 0.33 m
%       chord, twin-boom, 21 in tractor propeller).
%   'f550'   | 'hex' | 'hexacopter'       -> F550-class hexacopter
%       (550 mm motor-to-motor wheelbase, six 10 in propellers).
%
% NAME-VALUE OPTIONS
%   'scale'      uniform geometric scale factor (default: 1). Useful for
%                sweeping "how big does the target need to be".
%   'bodyGray'   reflectance of solid structure, 0 = black silhouette,
%                1 = as bright as the sky (default: 0.12). Airframes
%                seen from below against sky are near-silhouettes.
%   'propAlpha'  opacity of the spinning propeller discs (default: 0.35).
%                A spinning prop is partially transparent over a frame.
%
% BODY FRAME CONVENTION
%   x forward (nose), y left (port wing), z up. Origin at the nominal CG.
%   Rotation to world is R = Rz(yaw)*Ry(-pitch)*Rx(roll), see
%   ESTIMATE_UAV_ATTITUDE.
%
% OUTPUT FIELDS
%   .name, .type       'fixedwing' | 'multirotor'
%   .classId           integer class label (0 fixedwing, 1 multirotor),
%                      used by EXPORT_YOLO_DATASET
%   .verts             V-by-3 body-frame vertices (meters)
%   .faces             struct array with .idx (vertex indices, in order),
%                      .gray (0..1 reflectance), .alpha (0..1 opacity)
%   .lights            struct with .pos (M-by-3), .rgb (M-by-3),
%                      .intensity (M-by-1), .strobeHz (M-by-1),
%                      .duty (M-by-1)
%   .sizeRef           characteristic size (m), used for range-to-pixel
%                      sanity checks and reporting
%   .acoustics         suggested SIMULATE_UAV_ACOUSTICS settings
%                      (.sourceType, .sourceFreq) for this airframe
%
% NOTE
%   Dimensions are first-order representations of the real platforms,
%   good enough to set apparent angular size and rough silhouette shape.
%   They are not CAD-accurate outer mold lines.
%
% See also RENDER_UAV_PROJECTION, SIMULATE_UAV_CAMERAS, ESTIMATE_UAV_ATTITUDE

p = inputParser;
p.FunctionName = 'uav_platform';
addRequired(p, 'name');
addParameter(p, 'scale', 1);
addParameter(p, 'bodyGray', 0.12);
addParameter(p, 'propAlpha', 0.35);
parse(p, name, varargin{:});
opt = p.Results;

switch lower(strrep(strrep(name, '-', '_'), ' ', '_'))
    case {'swift', 'swift_uav', 'fixedwing', 'fixed_wing', 'swiftuav'}
        plat = build_swift(opt);
    case {'f550', 'f550_hex', 'hex', 'hexacopter', 'multirotor', 'hexrotor'}
        plat = build_f550(opt);
    otherwise
        error('uav_platform:badName', ...
            ['Unknown platform "%s". Use "swift" (fixed-wing) or ', ...
             '"f550" (hexacopter).'], name);
end

% --- apply uniform scale --------------------------------------------
plat.verts = plat.verts * opt.scale;
plat.lights.pos = plat.lights.pos * opt.scale;
plat.sizeRef = plat.sizeRef * opt.scale;
plat.scale = opt.scale;

end

% =====================================================================
function plat = build_swift(opt)
%BUILD_SWIFT SWIFT-UAV: 2.6 m span, 0.33 m chord, twin-boom fixed wing.

g = opt.bodyGray;
V = zeros(0, 3);
F = struct('idx', {}, 'gray', {}, 'alpha', {});

span    = 2.60;   % m, wingspan
chord   = 0.33;   % m, nominal chord
boomY   = 0.45;   % m, twin-boom lateral offset from centerline
boomW   = 0.020;  % m, 20x20 mm extruded aluminum profile
propD   = 0.533;  % m, 21 in propeller

% --- fuselage planform (nose at +x), slight taper at both ends -------
fus = [ 0.75  0.000  0.00;
        0.62  0.090  0.00;
       -0.30  0.090  0.00;
       -0.45  0.050  0.00;
       -0.45 -0.050  0.00;
       -0.30 -0.090  0.00;
        0.62 -0.090  0.00];
[V, F] = add_face(V, F, fus, g, 0.98);

% --- wing (rectangular planform, sits just above the fuselage) -------
wing = [ chord*0.5  span/2  0.02;
        -chord*0.5  span/2  0.02;
        -chord*0.5 -span/2  0.02;
         chord*0.5 -span/2  0.02];
[V, F] = add_face(V, F, wing, g*1.05, 0.98);

% --- twin tail booms --------------------------------------------------
for sgn = [1 -1]
    boom = [ 0.02  sgn*(boomY + boomW/2)  0.00;
            -1.05  sgn*(boomY + boomW/2)  0.00;
            -1.05  sgn*(boomY - boomW/2)  0.00;
             0.02  sgn*(boomY - boomW/2)  0.00];
    [V, F] = add_face(V, F, boom, g*0.9, 0.98);
end

% --- horizontal tail --------------------------------------------------
htail = [-0.90  0.46  0.05;
         -1.15  0.46  0.05;
         -1.15 -0.46  0.05;
         -0.90 -0.46  0.05];
[V, F] = add_face(V, F, htail, g, 0.98);

% --- vertical fins (nearly edge-on from directly below) ---------------
for sgn = [1 -1]
    fin = [-0.95  sgn*boomY  0.00;
           -1.15  sgn*boomY  0.00;
           -1.15  sgn*boomY  0.28;
           -1.00  sgn*boomY  0.28];
    [V, F] = add_face(V, F, fin, g*0.9, 0.98);
end

% --- tractor propeller disc (vertical plane -> edge-on from below) ----
prop = circle_pts([0.78 0 0], propD/2, 'x', 18);
[V, F] = add_face(V, F, prop, 0.45, opt.propAlpha);

plat = struct();
plat.name      = 'SWIFT-UAV (20 kg-class fixed wing)';
plat.type      = 'fixedwing';
plat.classId   = 0;
plat.verts     = V;
plat.faces     = F;
plat.sizeRef   = span;
plat.dims      = struct('span', span, 'chord', chord, 'length', 1.90, ...
                        'propDiameter', propD, 'mtowKg', 20);
% Suggested acoustics: ~2-blade 21 in prop at a 30-40 percent cruise
% throttle gives a blade-pass fundamental near 130 Hz.
plat.acoustics = struct('sourceType', 'rotor', 'sourceFreq', 130);
plat.lights    = struct( ...
    'pos',       [-0.05  span/2  0.02;    % port wingtip
                  -0.05 -span/2  0.02;    % starboard wingtip
                   0.00  0.00   -0.06], ... % belly beacon
    'rgb',       [0 1 0; 1 0 0; 1 1 1], ...
    'intensity', [0.7; 0.7; 1.0], ...
    'strobeHz',  [0; 0; 1.0], ...
    'duty',      [1; 1; 0.15]);

end

% =====================================================================
function plat = build_f550(opt)
%BUILD_F550 F550-class hexacopter: 550 mm wheelbase, six 10 in props.

g = opt.bodyGray;
V = zeros(0, 3);
F = struct('idx', {}, 'gray', {}, 'alpha', {});

armR   = 0.2765;  % m, center to motor (550 mm motor-to-motor)
armW   = 0.030;   % m, arm width
propD  = 0.254;   % m, 10 in propellers
hubR   = 0.100;   % m, center plate radius

% --- center plate (hexagon) ------------------------------------------
hub = circle_pts([0 0 0], hubR, 'z', 6);
[V, F] = add_face(V, F, hub, g, 0.98);

% --- six arms + six prop discs (hex-X layout, none along the nose) ---
angles = (30:60:330) * pi/180;
for a = angles
    ca = cos(a); sa = sin(a);
    r0 = hubR*0.7; r1 = armR;
    nx = -sa; ny = ca;  % unit normal to the arm, in plane
    arm = [ r0*ca + nx*armW/2, r0*sa + ny*armW/2, 0;
            r1*ca + nx*armW/2, r1*sa + ny*armW/2, 0;
            r1*ca - nx*armW/2, r1*sa - ny*armW/2, 0;
            r0*ca - nx*armW/2, r0*sa - ny*armW/2, 0];
    [V, F] = add_face(V, F, arm, g, 0.98);

    prop = circle_pts([r1*ca, r1*sa, 0.03], propD/2, 'z', 16);
    [V, F] = add_face(V, F, prop, 0.45, opt.propAlpha);
end

% --- landing skids (visible from below) -------------------------------
for sgn = [1 -1]
    skid = [ 0.16  sgn*0.11  -0.12;
            -0.16  sgn*0.11  -0.12;
            -0.16  sgn*0.09  -0.12;
             0.16  sgn*0.09  -0.12];
    [V, F] = add_face(V, F, skid, g*0.8, 0.98);
end

plat = struct();
plat.name      = 'F550-class hexacopter';
plat.type      = 'multirotor';
plat.classId   = 1;
plat.verts     = V;
plat.faces     = F;
plat.sizeRef   = 2*(armR + propD/2);   % ~0.81 m tip-to-tip
plat.dims      = struct('wheelbase', 2*armR, 'propDiameter', propD, ...
                        'tipToTip', 2*(armR + propD/2), 'numRotors', 6);
% Suggested acoustics: six 10 in props near hover give a blade-pass
% fundamental around 170 Hz with a dense harmonic stack.
plat.acoustics = struct('sourceType', 'rotor', 'sourceFreq', 170);

lightPos = zeros(6, 3);
for k = 1:6
    lightPos(k, :) = [armR*cos(angles(k)), armR*sin(angles(k)), -0.02];
end
plat.lights = struct( ...
    'pos',       lightPos, ...
    'rgb',       [1 0 0; 1 0 0; 1 1 1; 1 1 1; 1 1 1; 1 1 1], ...
    'intensity', 0.6*ones(6, 1), ...
    'strobeHz',  zeros(6, 1), ...
    'duty',      ones(6, 1));

end

% =====================================================================
function [V, F] = add_face(V, F, pts, gray, alpha)
%ADD_FACE Append a polygon (given as ordered 3D points) to the model.
n0 = size(V, 1);
V = [V; pts]; %#ok<AGROW>
k = numel(F) + 1;
F(k).idx   = n0 + (1:size(pts, 1));
F(k).gray  = gray;
F(k).alpha = alpha;
end

% =====================================================================
function pts = circle_pts(center, radius, normalAxis, n)
%CIRCLE_PTS Polygonal approximation of a circle normal to one body axis.
a = linspace(0, 2*pi, n+1)';
a(end) = [];
c = radius*cos(a);
s = radius*sin(a);
z = zeros(n, 1);
switch lower(normalAxis)
    case 'x', pts = [z, c, s];   % disc in the y-z plane (prop on a nose)
    case 'y', pts = [c, z, s];
    case 'z', pts = [c, s, z];   % disc in the x-y plane (rotor disc)
    otherwise
        error('uav_platform:badAxis', 'normalAxis must be x, y, or z.');
end
pts = pts + repmat(center(:)', n, 1);
end
