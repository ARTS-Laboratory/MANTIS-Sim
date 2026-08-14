function scene = simulate_uav_scene(nodeXY, uavXYZ, t, varargin)
%SIMULATE_UAV_SCENE Joint acoustic and visual simulation of a UAV overflight.
%
%   scene = SIMULATE_UAV_SCENE(nodeXY, uavXYZ, t) simulates a network of
%   ground sensor nodes, each carrying a microphone and an upward-facing
%   fisheye camera, observing a UAV in flight. It returns one struct
%   holding the audio-rate microphone signals, the low-rate synchronized
%   camera frames, and the ground truth that ties them together.
%
%   The acoustic path is unchanged from v0.1: this function calls
%   SIMULATE_UAV_ACOUSTICS with the same model and parameters. The
%   vision path is added alongside it and shares the node geometry, the
%   trajectory, and the time base.
%
% REQUIRED INPUTS
%   nodeXY : N-by-2 sensor node (x,y) coordinates in meters. Each node
%            is one deployable package: a microphone at ground level and
%            a camera looking straight up.
%   uavXYZ : T-by-3 UAV positions in meters at the times in t.
%   t      : T-by-1 audio-rate time vector in seconds.
%
% OPTIONAL NAME-VALUE PAIRS
%   Platform
%     'platform'        'swift' (default) | 'f550' | struct
%     'platformScale'   geometric scale factor (default 1)
%     'attitude'        T-by-3 [yaw pitch roll], default derived from
%                       the path by ESTIMATE_UAV_ATTITUDE
%
%   Acoustics (passed straight through to SIMULATE_UAV_ACOUSTICS)
%     'fs', 'c', 'alpha', 'snrDb', 'enableDoppler', 'enableReflection',
%     'reflectionGain', 'sourceType', 'sourceFreq', 'sourceWaveform',
%     'noiseType'
%     If 'sourceType' and 'sourceFreq' are left at their defaults, the
%     platform's suggested values are used (130 Hz rotor-like for the
%     fixed wing, 170 Hz for the hexacopter).
%
%   Vision (passed through to SIMULATE_UAV_CAMERAS)
%     'fps', 'imageSize', 'fovDeg', 'projection', 'camHeight',
%     'camYawDeg', 'channels', 'sky', 'navLights', 'exposureTime',
%     'subFrames', 'gain', 'readNoise', 'fullWell', 'superSample',
%     'edgeDivisions'
%
%   Network resilience
%     'deadNodes'    indices (or logical mask) of nodes that are
%                    completely offline: microphone and camera both.
%                    Use this for the "can the network still track the
%                    target with node k removed" experiments.
%     'deadMics'     nodes whose microphone alone has failed
%     'deadCameras'  nodes whose camera alone has failed
%     'dropoutRate'  probability that a whole node is offline, applied
%                    on top of the explicit lists (default 0)
%
%   Bookkeeping
%     'seed'         random seed (default 42)
%     'verbose'      print a summary (default true)
%
% OUTPUT scene
%   .nodes    .XY, .camHeight, .deadNodes, .deadMics, .deadCameras
%   .audio    .Y (numSamples-by-N), .t, .fs, .meta, .extras
%   .video    .frames (H-by-W-by-C-by-K-by-N uint8), .meta
%   .truth    .uavXYZ, .attitude, .t, .platform, .rangeToNodes
%   .config   the resolved option set
%   .createdAt
%
% TIME ALIGNMENT
%   Audio runs at fs. Frames are snapped onto the audio sample grid, so
%   frame k at every node corresponds exactly to audio sample
%   scene.video.meta.frameIdx(k), at time scene.video.meta.frameTimes(k).
%   No resampling is needed to fuse the two streams.
%
% EXAMPLE
%   fs     = 4000;
%   t      = (0:1/fs:24)';
%   nodeXY = generate_node_array(6, 'ring', 'radius', 50);
%   uavXYZ = generate_uav_trajectory(t, 'straight', ...
%       'startXY', [-300 -40], 'endXY', [300 40], 'altitude', 80);
%   scene  = simulate_uav_scene(nodeXY, uavXYZ, t, ...
%       'platform', 'swift', 'sky', 'cloudy', 'fps', 1, 'snrDb', 10);
%   plot_scene_results(scene);
%
% See also SIMULATE_UAV_ACOUSTICS, SIMULATE_UAV_CAMERAS, PLOT_SCENE_RESULTS,
%          SAVE_SCENE, EXPORT_YOLO_DATASET, GENERATE_NODE_ARRAY

% ---------------------------------------------------------------------
% 1. Parse
% ---------------------------------------------------------------------
p = inputParser;
p.FunctionName = 'simulate_uav_scene';
addRequired(p, 'nodeXY');
addRequired(p, 'uavXYZ');
addRequired(p, 't');

% platform
addParameter(p, 'platform', 'swift');
addParameter(p, 'platformScale', 1);
addParameter(p, 'attitude', []);

% acoustics
addParameter(p, 'fs', []);
addParameter(p, 'c', 343);
addParameter(p, 'alpha', 1);
addParameter(p, 'snrDb', 15);
addParameter(p, 'enableDoppler', true);
addParameter(p, 'enableReflection', false);
addParameter(p, 'reflectionGain', 0.4);
addParameter(p, 'sourceType', '');
addParameter(p, 'sourceFreq', []);
addParameter(p, 'sourceWaveform', []);
addParameter(p, 'noiseType', 'white');

% vision
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
addParameter(p, 'superSample', 3);
addParameter(p, 'edgeDivisions', 4);

% network
addParameter(p, 'deadNodes', []);
addParameter(p, 'deadMics', []);
addParameter(p, 'deadCameras', []);
addParameter(p, 'dropoutRate', 0);

% bookkeeping
addParameter(p, 'seed', 42);
addParameter(p, 'verbose', true);

parse(p, nodeXY, uavXYZ, t, varargin{:});
opt = p.Results;

t = t(:);
numNodes = size(nodeXY, 1);
if isempty(nodeXY) || size(nodeXY, 2) ~= 2
    error('simulate_uav_scene:badNodeXY', ...
        'nodeXY must be an N-by-2 matrix of (x,y) node coordinates.');
end
if opt.dropoutRate < 0 || opt.dropoutRate > 1
    error('simulate_uav_scene:badDropout', 'dropoutRate must be in [0,1].');
end

% --- platform and its suggested acoustics -----------------------------
if isstruct(opt.platform)
    plat = opt.platform;
else
    plat = uav_platform(opt.platform, 'scale', opt.platformScale);
end
if isempty(opt.sourceType)
    opt.sourceType = plat.acoustics.sourceType;
end
if isempty(opt.sourceFreq)
    opt.sourceFreq = plat.acoustics.sourceFreq;
end

% --- attitude ---------------------------------------------------------
if isempty(opt.attitude)
    att = estimate_uav_attitude(uavXYZ, t, 'mode', plat.type);
else
    att = opt.attitude;
end

% ---------------------------------------------------------------------
% 2. Resolve which nodes, microphones, and cameras are alive
% ---------------------------------------------------------------------
deadNode = to_mask(opt.deadNodes, numNodes, 'deadNodes');
if opt.dropoutRate > 0
    rng(opt.seed);
    deadNode = deadNode | (rand(1, numNodes) < opt.dropoutRate);
end
deadMic = deadNode | to_mask(opt.deadMics, numNodes, 'deadMics');
deadCam = deadNode | to_mask(opt.deadCameras, numNodes, 'deadCameras');

% ---------------------------------------------------------------------
% 3. Acoustics (v0.1 pipeline, untouched)
% ---------------------------------------------------------------------
acousticArgs = { ...
    'c', opt.c, 'alpha', opt.alpha, 'snrDb', opt.snrDb, ...
    'enableDoppler', opt.enableDoppler, ...
    'enableReflection', opt.enableReflection, ...
    'reflectionGain', opt.reflectionGain, ...
    'sourceType', opt.sourceType, 'sourceFreq', opt.sourceFreq, ...
    'sourceWaveform', opt.sourceWaveform, ...
    'noiseType', opt.noiseType, ...
    'dropoutRate', 0, ...    % node dropout is handled here, coherently
    'seed', opt.seed};
if ~isempty(opt.fs)
    acousticArgs = [acousticArgs, {'fs', opt.fs}];
end

[Y, tOut, meta, extras] = simulate_uav_acoustics(nodeXY, uavXYZ, t, acousticArgs{:});

% Dead microphones flatline, matching the v0.1 dropout convention.
if any(deadMic)
    Y(:, deadMic) = 0;
    extras.cleanY(:, deadMic) = 0;
end
meta.dropout = struct('rate', opt.dropoutRate, 'deadMics', deadMic);

% ---------------------------------------------------------------------
% 4. Vision
% ---------------------------------------------------------------------
[frames, camMeta] = simulate_uav_cameras(nodeXY, uavXYZ, t, ...
    'platform', plat, 'attitude', att, ...
    'fps', opt.fps, 'imageSize', opt.imageSize, 'fovDeg', opt.fovDeg, ...
    'projection', opt.projection, 'camHeight', opt.camHeight, ...
    'camYawDeg', opt.camYawDeg, 'channels', opt.channels, ...
    'sky', opt.sky, 'navLights', opt.navLights, ...
    'exposureTime', opt.exposureTime, 'subFrames', opt.subFrames, ...
    'gain', opt.gain, 'readNoise', opt.readNoise, 'fullWell', opt.fullWell, ...
    'superSample', opt.superSample, 'edgeDivisions', opt.edgeDivisions, ...
    'deadNodes', deadCam, 'seed', opt.seed, 'verbose', opt.verbose);

% ---------------------------------------------------------------------
% 5. Package
% ---------------------------------------------------------------------
rangeToNodes = zeros(numel(tOut), numNodes);
for n = 1:numNodes
    d = uavXYZ - repmat([nodeXY(n,:), opt.camHeight], numel(tOut), 1);
    rangeToNodes(:, n) = sqrt(sum(d.^2, 2));
end

scene = struct();
scene.version = 'uav-emulator v0.2 (acoustic + vision)';
scene.nodes = struct('XY', nodeXY, 'camHeight', opt.camHeight, ...
    'deadNodes', deadNode, 'deadMics', deadMic, 'deadCameras', deadCam);
scene.audio = struct('Y', Y, 't', tOut, 'fs', meta.fs, ...
    'meta', meta, 'extras', extras);
scene.video = struct('frames', frames, 'meta', camMeta);
scene.truth = struct('uavXYZ', uavXYZ, 'attitude', att, 't', tOut, ...
    'platform', plat, 'rangeToNodes', rangeToNodes);
scene.config = opt;
scene.createdAt = datestr(now);

if opt.verbose
    print_summary(scene);
end

end

% =====================================================================
function mask = to_mask(spec, numNodes, name)
mask = false(1, numNodes);
if isempty(spec), return; end
if islogical(spec)
    if numel(spec) ~= numNodes
        error('simulate_uav_scene:badMask', ...
            'Logical %s must have one entry per node.', name);
    end
    mask = spec(:)';
else
    if any(spec < 1) || any(spec > numNodes)
        error('simulate_uav_scene:badIndex', ...
            '%s indices must lie in 1..%d.', name, numNodes);
    end
    mask(spec) = true;
end
end

% =====================================================================
function print_summary(scene)
tr = scene.video.meta.truth;
nNodes = size(scene.nodes.XY, 1);
fprintf('\n--- scene summary -------------------------------------------\n');
fprintf('platform      : %s\n', scene.truth.platform.name);
fprintf('nodes         : %d (%d mic dead, %d camera dead)\n', ...
    nNodes, sum(scene.nodes.deadMics), sum(scene.nodes.deadCameras));
fprintf('audio         : %d samples at %g Hz, %.1f s\n', ...
    size(scene.audio.Y, 1), scene.audio.fs, scene.audio.t(end) - scene.audio.t(1));
fprintf('video         : %d frames at %g fps, %dx%d px, %d channel(s)\n', ...
    numel(scene.video.meta.frameTimes), scene.video.meta.fps, ...
    scene.video.meta.imageSize(1), scene.video.meta.imageSize(2), ...
    scene.video.meta.channels);
fprintf('sky           : %s\n', scene.video.meta.sky.type);
fprintf('closest range : %.1f m\n', min(scene.truth.rangeToNodes(:)));
alive = ~scene.nodes.deadCameras;
if any(alive)
    pa = tr.pixelArea(:, alive);
    fprintf('target pixels : max %.1f, median over in-FOV frames %.2f\n', ...
        max(pa(:)), median(pa(pa > 0)));
    resolvable = sum(any(pa > 0.5, 2));
    fprintf('frames with a resolvable target on at least one camera: %d/%d\n', ...
        resolvable, size(pa, 1));
end
fprintf('-------------------------------------------------------------\n\n');
end
