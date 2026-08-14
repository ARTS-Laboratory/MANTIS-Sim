function [frames, camMeta] = simulate_uav_cameras(nodeXY, uavXYZ, t, varargin)
%SIMULATE_UAV_CAMERAS Synthetic upward-looking fisheye frames of a UAV.
%
%   [frames, camMeta] = SIMULATE_UAV_CAMERAS(nodeXY, uavXYZ, t) renders,
%   for every ground sensor node, what an upward-facing fisheye camera
%   at that node would see as the UAV flies overhead. Every node is
%   triggered at the same instants, so the frames form a synchronized
%   multi-view set suitable for stereo or network-level fusion.
%
% REQUIRED INPUTS
%   nodeXY : N-by-2 sensor node (x,y) ground coordinates in meters. Use
%            the same array you pass to SIMULATE_UAV_ACOUSTICS as micXY
%            so that each node carries a co-located microphone and
%            camera.
%   uavXYZ : T-by-3 UAV positions in meters, sampled at times t.
%   t      : T-by-1 audio-rate time vector (seconds).
%
% OPTIONAL NAME-VALUE PAIRS
%   Platform and motion
%     'platform'      'swift' (default) | 'f550' | struct from UAV_PLATFORM
%     'attitude'      T-by-3 [yaw pitch roll] in radians. Default is
%                     derived from the trajectory by ESTIMATE_UAV_ATTITUDE.
%     'platformScale' geometric scale factor on the airframe (default 1)
%   Camera and array
%     'fps'           frame rate in Hz (default 1). Frames are placed on
%                     the audio sample grid so the two streams align.
%     'imageSize'     [rows cols] in pixels (default [240 240])
%     'fovDeg'        full fisheye field of view (default 185)
%     'projection'    'equidistant' (default) | 'equisolid' | 'orthographic'
%     'camHeight'     camera height above ground in meters (default 0.15)
%     'camYawDeg'     scalar or N-by-1 camera azimuth mounting angles
%                     (default 0)
%     'channels'      1 for monochrome (default, matching the ground
%                     camera packages) or 3 for RGB
%   Scene
%     'sky'           'day' (default) | 'cloudy' | 'dusk' | 'night', or a
%                     sky options struct (see RENDER_SKY_BACKGROUND)
%     'navLights'     true/false. Default true for dusk and night.
%   Sensor
%     'exposureTime'  seconds (default 0.002). With 'subFrames' > 1 this
%                     produces motion blur.
%     'subFrames'     renders averaged across the exposure (default 1)
%     'gain'          analog gain. Default 1 day/cloudy, 4 dusk, 12 night.
%     'readNoise'     read noise in electrons rms (default 6)
%     'fullWell'      electrons at full scale (default 8000)
%   Network and reproducibility
%     'deadNodes'     indices of nodes whose camera is dead, or a logical
%                     1-by-N mask (default none). Frames are returned as
%                     all zeros for those nodes.
%     'seed'          random seed (default 42)
%     'superSample'   subpixel samples per axis when rasterizing (3)
%     'edgeDivisions' polygon edge subdivisions before projection (4)
%     'verbose'       print progress (default true)
%
% OUTPUTS
%   frames  : H-by-W-by-C-by-K-by-N uint8 array. frames(:,:,:,k,n) is
%             frame k from node n. All nodes share the same frame times.
%   camMeta : struct with
%       .cams          1-by-N camera structs (MAKE_FISHEYE_CAMERA)
%       .frameTimes    K-by-1 frame times in seconds
%       .frameIdx      K-by-1 indices into the audio time vector t
%       .fps, .imageSize, .channels, .fovDeg
%       .platform      the platform struct actually used
%       .attitude      T-by-3 attitude used
%       .sky           resolved sky options
%       .photOpts      sensor model options
%       .deadNodes     1-by-N logical
%       .truth         ground truth per frame and node (see below)
%       .seed, .createdAt
%
%   camMeta.truth fields, each K-by-N unless noted:
%       .u, .v         pixel coordinates of the UAV body origin
%       .range         slant range from node to UAV (m)
%       .pixelArea     composited target coverage in pixels
%       .angularSize   apparent angular size of the airframe (deg)
%       .inFOV         logical, target inside the field of view
%       .bbox          K-by-N-by-4, [uMin vMin width height] in pixels
%
% EXAMPLE
%   t      = (0:1/4000:20)';
%   nodeXY = generate_node_array(6, 'ring', 'radius', 50);
%   uavXYZ = generate_uav_trajectory(t, 'straight', ...
%       'startXY', [-250 -40], 'endXY', [250 40], 'altitude', 80);
%   [frames, camMeta] = simulate_uav_cameras(nodeXY, uavXYZ, t, ...
%       'platform', 'swift', 'fps', 1, 'sky', 'cloudy');
%
% See also SIMULATE_UAV_SCENE, UAV_PLATFORM, MAKE_FISHEYE_CAMERA,
%          RENDER_SKY_BACKGROUND, RENDER_UAV_PROJECTION, ADD_IMAGE_NOISE

% ---------------------------------------------------------------------
% 1. Parse and validate
% ---------------------------------------------------------------------
p = inputParser;
p.FunctionName = 'simulate_uav_cameras';
addRequired(p, 'nodeXY');
addRequired(p, 'uavXYZ');
addRequired(p, 't');
addParameter(p, 'platform', 'swift');
addParameter(p, 'platformScale', 1);
addParameter(p, 'attitude', []);
addParameter(p, 'fps', 1);
addParameter(p, 'imageSize', [240 240]);
addParameter(p, 'fovDeg', 185);
addParameter(p, 'projection', 'equidistant');
addParameter(p, 'camHeight', 0.15);
addParameter(p, 'camYawDeg', 0);
addParameter(p, 'channels', 1);
addParameter(p, 'sky', 'day');
addParameter(p, 'navLights', []);
addParameter(p, 'exposureTime', 0.002);
addParameter(p, 'subFrames', 1);
addParameter(p, 'gain', []);
addParameter(p, 'readNoise', 6);
addParameter(p, 'fullWell', 8000);
addParameter(p, 'deadNodes', []);
addParameter(p, 'seed', 42);
addParameter(p, 'superSample', 3);
addParameter(p, 'edgeDivisions', 4);
addParameter(p, 'verbose', true);
parse(p, nodeXY, uavXYZ, t, varargin{:});
opt = p.Results;

t = t(:);
numSamples = numel(t);

if isempty(nodeXY) || size(nodeXY, 2) ~= 2
    error('simulate_uav_cameras:badNodeXY', ...
        'nodeXY must be an N-by-2 matrix of (x,y) node coordinates.');
end
if ~isnumeric(nodeXY) || any(~isfinite(nodeXY(:)))
    error('simulate_uav_cameras:badNodeXY', ...
        'nodeXY must contain finite numeric values.');
end
if isempty(uavXYZ) || size(uavXYZ, 2) ~= 3
    error('simulate_uav_cameras:badUavXYZ', ...
        'uavXYZ must be a T-by-3 matrix of (x,y,z) UAV positions.');
end
if size(uavXYZ, 1) ~= numSamples
    error('simulate_uav_cameras:sizeMismatch', ...
        'Rows of uavXYZ (%d) must match numel(t) (%d).', ...
        size(uavXYZ, 1), numSamples);
end
if numSamples < 2
    error('simulate_uav_cameras:tooFewSamples', ...
        't must contain at least 2 samples.');
end
if any(diff(t) <= 0)
    error('simulate_uav_cameras:tNotMonotonic', 't must be strictly increasing.');
end
if opt.fps <= 0 || ~isfinite(opt.fps)
    error('simulate_uav_cameras:badFps', 'fps must be a positive finite scalar.');
end
if ~any(opt.channels == [1 3])
    error('simulate_uav_cameras:badChannels', 'channels must be 1 or 3.');
end

numNodes = size(nodeXY, 1);
H = opt.imageSize(1);
W = opt.imageSize(2);
C = opt.channels;

rng(opt.seed);

% --- platform ---------------------------------------------------------
if isstruct(opt.platform)
    plat = opt.platform;
else
    plat = uav_platform(opt.platform, 'scale', opt.platformScale);
end

% --- attitude ---------------------------------------------------------
if isempty(opt.attitude)
    att = estimate_uav_attitude(uavXYZ, t, 'mode', plat.type);
else
    att = opt.attitude;
    if size(att, 1) ~= numSamples || size(att, 2) ~= 3
        error('simulate_uav_cameras:badAttitude', ...
            'attitude must be a T-by-3 array of [yaw pitch roll] in radians.');
    end
end

% --- sky --------------------------------------------------------------
if ischar(opt.sky)
    sky = struct('type', opt.sky);
else
    sky = opt.sky;
    if ~isfield(sky, 'type'), sky.type = 'day'; end
end
sky.channels = C;

isDark = any(strcmpi(sky.type, {'night', 'dusk'}));
if isempty(opt.navLights), opt.navLights = isDark; end
if isempty(opt.gain)
    switch lower(sky.type)
        case 'night', opt.gain = 12;
        case 'dusk',  opt.gain = 4;
        otherwise,    opt.gain = 1;
    end
end

photOpts = struct('fullWell', opt.fullWell, 'readNoise', opt.readNoise, ...
    'gain', opt.gain, 'blackLevel', 4, 'quantize', true);

% --- dead cameras -----------------------------------------------------
dead = false(1, numNodes);
if ~isempty(opt.deadNodes)
    if islogical(opt.deadNodes)
        if numel(opt.deadNodes) ~= numNodes
            error('simulate_uav_cameras:badDeadNodes', ...
                'Logical deadNodes must have one entry per node.');
        end
        dead = opt.deadNodes(:)';
    else
        if any(opt.deadNodes < 1) || any(opt.deadNodes > numNodes)
            error('simulate_uav_cameras:badDeadNodes', ...
                'deadNodes indices must lie in 1..%d.', numNodes);
        end
        dead(opt.deadNodes) = true;
    end
end

% --- frame schedule ---------------------------------------------------
fsAudio = 1 / median(diff(t));
step = max(1, round(fsAudio / opt.fps));
frameIdx = (1:step:numSamples)';
frameTimes = t(frameIdx);
K = numel(frameIdx);

% --- cameras ----------------------------------------------------------
yawDeg = opt.camYawDeg;
if isscalar(yawDeg), yawDeg = yawDeg * ones(numNodes, 1); end
for n = numNodes:-1:1
    cams(n) = make_fisheye_camera([nodeXY(n,1), nodeXY(n,2), opt.camHeight], ...
        'imageSize', [H W], 'fovDeg', opt.fovDeg, 'yawDeg', yawDeg(n), ...
        'projection', opt.projection);
end

% ---------------------------------------------------------------------
% 2. Render
% ---------------------------------------------------------------------
frames = zeros(H, W, C, K, numNodes, 'uint8');

truth = struct();
truth.u = nan(K, numNodes);
truth.v = nan(K, numNodes);
truth.range = nan(K, numNodes);
truth.pixelArea = zeros(K, numNodes);
truth.angularSize = nan(K, numNodes);
truth.inFOV = false(K, numNodes);
truth.bbox = nan(K, numNodes, 4);

nSub = max(1, round(opt.subFrames));
subOffsets = ((1:nSub) - (nSub + 1)/2) / nSub * opt.exposureTime;
centerSub = ceil(nSub/2);

ropts = struct('superSample', opt.superSample, ...
    'edgeDivisions', opt.edgeDivisions, 'navLights', opt.navLights, ...
    'time', 0, 'lightGain', 1);

if opt.verbose
    fprintf('simulate_uav_cameras: %d nodes x %d frames at %g fps, %dx%d px\n', ...
        numNodes, K, opt.fps, H, W);
end

for n = 1:numNodes
    if dead(n)
        if opt.verbose
            fprintf('  node %2d: camera dead, frames zeroed\n', n);
        end
        continue;
    end

    cam = cams(n);
    skyState = sky;
    rng(opt.seed + 1000*n);   % per-node sensor noise, reproducible

    bgCached = [];
    staticSky = false;
    for k = 1:K
        tk = frameTimes(k);
        if isempty(bgCached) || ~staticSky
            [bg, skyState] = render_sky_background(cam, skyState, tk);
            if k == 1
                % Nothing in the sky moves unless cloud is present and
                % drifting, so in the common case one render per camera
                % is enough. Sensor noise still differs frame to frame.
                staticSky = (skyState.cloudCover <= 0) || ...
                    all(skyState.cloudDrift == 0);
                bgCached = bg;
            end
        else
            bg = bgCached;
        end

        acc = zeros(H, W, C);
        frameTruth = [];
        for sIdx = 1:nSub
            ts = tk + subOffsets(sIdx);
            [posS, attS] = pose_at(ts, t, uavXYZ, att);
            R = rot_body_to_world(attS(1), attS(2), attS(3));
            ropts.time = ts;
            [imgS, truthS] = render_uav_projection(bg, cam, plat, posS, R, ropts);
            acc = acc + imgS;
            if sIdx == centerSub
                frameTruth = truthS;
            end
        end
        img = acc / nSub;

        % Lens illumination falloff plus the circular image mask.
        img = img .* repmat(cam.vignette, [1 1 C]);

        frames(:,:,:,k,n) = add_image_noise(img, photOpts);

        truth.u(k,n) = frameTruth.u;
        truth.v(k,n) = frameTruth.v;
        truth.range(k,n) = frameTruth.range;
        truth.pixelArea(k,n) = frameTruth.pixelArea;
        truth.angularSize(k,n) = frameTruth.angularSize;
        truth.inFOV(k,n) = frameTruth.inFOV;
        truth.bbox(k,n,:) = frameTruth.bbox;
    end

    if opt.verbose
        seen = sum(truth.pixelArea(:,n) > 0.5);
        fprintf('  node %2d: %3d/%3d frames with a resolvable target, min range %.1f m\n', ...
            n, seen, K, min(truth.range(:,n)));
    end
end

% ---------------------------------------------------------------------
% 3. Package metadata
% ---------------------------------------------------------------------
camMeta = struct();
camMeta.cams = cams;
camMeta.nodeXY = nodeXY;
camMeta.camHeight = opt.camHeight;
camMeta.frameTimes = frameTimes;
camMeta.frameIdx = frameIdx;
camMeta.fps = fsAudio / step;          % actual rate after grid snapping
camMeta.requestedFps = opt.fps;
camMeta.imageSize = [H W];
camMeta.channels = C;
camMeta.fovDeg = opt.fovDeg;
camMeta.projection = opt.projection;
camMeta.platform = plat;
camMeta.attitude = att;
camMeta.sky = sky;
camMeta.navLights = opt.navLights;
camMeta.exposureTime = opt.exposureTime;
camMeta.subFrames = nSub;
camMeta.photOpts = photOpts;
camMeta.deadNodes = dead;
camMeta.truth = truth;
camMeta.seed = opt.seed;
camMeta.pixPerDeg = cams(1).pixPerRad * pi/180;
camMeta.createdAt = datestr(now);

end

% =====================================================================
function [pos, att1] = pose_at(ts, t, uavXYZ, att)
%POSE_AT Interpolate UAV position and attitude at an arbitrary time.
ts = max(t(1), min(t(end), ts));
pos = [interp1(t, uavXYZ(:,1), ts, 'linear'), ...
       interp1(t, uavXYZ(:,2), ts, 'linear'), ...
       interp1(t, uavXYZ(:,3), ts, 'linear')];
att1 = [interp1(t, att(:,1), ts, 'linear'), ...
        interp1(t, att(:,2), ts, 'linear'), ...
        interp1(t, att(:,3), ts, 'linear')];
end

% =====================================================================
function R = rot_body_to_world(yaw, pitch, roll)
%ROT_BODY_TO_WORLD Body (x forward, y left, z up) to world rotation.
%   R = Rz(yaw) * Ry(-pitch) * Rx(roll), so that positive pitch is nose
%   up and positive roll puts the right wing down.
cy = cos(yaw);   sy = sin(yaw);
cp = cos(-pitch); sp = sin(-pitch);
cr = cos(roll);  sr = sin(roll);
Rz = [cy -sy 0; sy cy 0; 0 0 1];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rx = [1 0 0; 0 cr -sr; 0 sr cr];
R = Rz * Ry * Rx;
end
