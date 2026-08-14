function figs = plot_scene_results(scene, varargin)
%PLOT_SCENE_RESULTS Visualize a joint acoustic and visual UAV scene.
%
%   PLOT_SCENE_RESULTS(scene) creates two figures:
%     1. Scene overview: node layout and UAV ground track, the audio
%        waveform at one node with the camera trigger instants marked,
%        slant range to every node, and the target track in the image
%        plane of one camera.
%     2. Camera montage: the synchronized frame from every node at one
%        instant, with the ground-truth bounding box overlaid.
%
% NAME-VALUE OPTIONS
%   'frameIndex'   which frame to show in the montage. Default is the
%                  frame at which the target is largest on any camera,
%                  which is usually the most informative one.
%   'audioNode'    node whose waveform is plotted (default 1, or the
%                  first node with a live microphone)
%   'trackNode'    node whose image-plane track is plotted (default 1)
%   'showBoxes'    overlay ground-truth boxes on the montage (default true)
%   'maxNodes'     cap on montage panels (default 12)
%
% OUTPUT
%   figs : handles to the created figures
%
% See also SIMULATE_UAV_SCENE, PLOT_SIMULATION_RESULTS

p = inputParser;
p.FunctionName = 'plot_scene_results';
addParameter(p, 'frameIndex', []);
addParameter(p, 'audioNode', []);
addParameter(p, 'trackNode', 1);
addParameter(p, 'showBoxes', true);
addParameter(p, 'maxNodes', 12);
parse(p, varargin{:});
opt = p.Results;

nodeXY = scene.nodes.XY;
numNodes = size(nodeXY, 1);
vm = scene.video.meta;
tr = vm.truth;
frames = scene.video.frames;
K = numel(vm.frameTimes);

if isempty(opt.audioNode)
    live = find(~scene.nodes.deadMics, 1, 'first');
    if isempty(live), live = 1; end
    opt.audioNode = live;
end
if isempty(opt.frameIndex)
    [~, opt.frameIndex] = max(max(tr.pixelArea, [], 2));
    if isempty(opt.frameIndex) || ~isfinite(opt.frameIndex)
        opt.frameIndex = max(1, round(K/2));
    end
end
opt.frameIndex = max(1, min(K, opt.frameIndex));

figs = [];

% =====================================================================
% Figure 1: scene overview
% =====================================================================
f1 = figure('Name', 'UAV Scene - Overview', 'Color', 'w');
figs(end+1) = f1;

% --- (1) geometry ------------------------------------------------------
subplot(2, 2, 1);
plot(scene.truth.uavXYZ(:,1), scene.truth.uavXYZ(:,2), 'b-', 'LineWidth', 1.5);
hold on;
liveIdx = find(~scene.nodes.deadNodes);
deadIdx = find(scene.nodes.deadNodes);
if ~isempty(liveIdx)
    scatter(nodeXY(liveIdx,1), nodeXY(liveIdx,2), 60, 'r', 'filled');
end
if ~isempty(deadIdx)
    scatter(nodeXY(deadIdx,1), nodeXY(deadIdx,2), 60, [0.5 0.5 0.5], 'x', ...
        'LineWidth', 1.5);
end
for i = 1:numNodes
    text(nodeXY(i,1), nodeXY(i,2), sprintf('  N%d', i), 'FontSize', 8);
end
plot(scene.truth.uavXYZ(1,1), scene.truth.uavXYZ(1,2), 'go', 'MarkerFaceColor', 'g');
plot(scene.truth.uavXYZ(end,1), scene.truth.uavXYZ(end,2), 'ks', 'MarkerFaceColor', 'k');
xlabel('X (m)'); ylabel('Y (m)');
title('Node layout and UAV ground track');
axis equal; grid on; hold off;

% --- (2) audio with camera triggers ------------------------------------
subplot(2, 2, 2);
plot(scene.audio.t, scene.audio.Y(:, opt.audioNode), 'Color', [0 0.4 0.8]);
hold on;
yl = ylim;
for k = 1:K
    plot([vm.frameTimes(k) vm.frameTimes(k)], yl, 'Color', [0.85 0.4 0.1 ], ...
        'LineWidth', 0.5);
end
xlabel('Time (s)'); ylabel('Amplitude');
title(sprintf('Microphone at node %d, camera triggers marked', opt.audioNode));
grid on; hold off;

% --- (3) range to each node --------------------------------------------
subplot(2, 2, 3);
plot(scene.truth.t, scene.truth.rangeToNodes, 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Slant range (m)');
title('UAV range to each node');
grid on;

% --- (4) image-plane track ---------------------------------------------
subplot(2, 2, 4);
n = max(1, min(numNodes, opt.trackNode));
cam = vm.cams(n);
th = linspace(0, 2*pi, 200);
plot(cam.cx + cam.imageRadius*cos(th), cam.cy + cam.imageRadius*sin(th), ...
    'k-', 'LineWidth', 1);
hold on;
vis = tr.inFOV(:, n);
scatter(tr.u(vis, n), tr.v(vis, n), 25, vm.frameTimes(vis), 'filled');
cb = colorbar; ylabel(cb, 'Time (s)');
xlabel('u (px)'); ylabel('v (px)');
title(sprintf('Target track in the image of node %d', n));
axis equal; axis ij;
xlim([0.5, vm.imageSize(2)+0.5]); ylim([0.5, vm.imageSize(1)+0.5]);
grid on; hold off;

add_super_title(sprintf('%s over %d nodes, %s sky', ...
    scene.truth.platform.name, numNodes, vm.sky.type));

% =====================================================================
% Figure 2: synchronized camera montage
% =====================================================================
nShow = min(numNodes, opt.maxNodes);
nCols = ceil(sqrt(nShow));
nRows = ceil(nShow / nCols);

f2 = figure('Name', 'UAV Scene - Camera Montage', 'Color', 'w');
figs(end+1) = f2;
colormap(gray(256));

k = opt.frameIndex;
for n = 1:nShow
    subplot(nRows, nCols, n);
    frame = frames(:, :, :, k, n);
    if size(frame, 3) == 3
        image(frame);
    else
        imagesc(frame);
        caxis([0 255]);
    end
    axis image off;
    if scene.nodes.deadCameras(n)
        title(sprintf('N%d (camera offline)', n), 'FontSize', 9, ...
            'Color', [0.6 0.2 0.2]);
        continue;
    end
    if opt.showBoxes && tr.inFOV(k, n) && all(isfinite(squeeze(tr.bbox(k,n,:))))
        bb = squeeze(tr.bbox(k, n, :))';
        pad = 3;
        rectangle('Position', [bb(1)-pad, bb(2)-pad, bb(3)+2*pad, bb(4)+2*pad], ...
            'EdgeColor', [0.1 0.9 0.2], 'LineWidth', 1);
    end
    title(sprintf('N%d  r=%.0f m  %.1f px', n, tr.range(k,n), tr.pixelArea(k,n)), ...
        'FontSize', 9);
end
add_super_title(sprintf('Synchronized frames at t = %.2f s (frame %d of %d)', ...
    vm.frameTimes(k), k, K));

end

% =====================================================================
function add_super_title(str)
%ADD_SUPER_TITLE Figure-level title, using sgtitle where available.
if exist('sgtitle', 'file') || exist('sgtitle', 'builtin')
    sgtitle(str);
else
    annotation('textbox', [0 0.94 1 0.06], 'String', str, ...
        'HorizontalAlignment', 'center', 'EdgeColor', 'none', ...
        'FontWeight', 'bold');
end
end
