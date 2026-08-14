function manifest = export_yolo_dataset(scene, outDir, varargin)
%EXPORT_YOLO_DATASET Write frames and YOLO labels from a simulated scene.
%
%   manifest = EXPORT_YOLO_DATASET(scene, outDir) writes every rendered
%   frame as a PNG under outDir/images and a matching YOLO-format label
%   file under outDir/labels, so a detector can be trained on synthetic
%   data before any field deployment. The label geometry comes from the
%   renderer, so it is exact rather than hand-drawn.
%
%   Label format, one line per object (here at most one per frame):
%       <classId> <xCenter> <yCenter> <width> <height>
%   with all four box values normalized by image width and height.
%
% NAME-VALUE OPTIONS
%   'minPixelArea'  skip frames where the target covers fewer than this
%                   many pixels (default 1.5). Frames below the
%                   threshold are still written as images when
%                   'keepEmpty' is true, with an empty label file, which
%                   is how negatives are supplied to YOLO.
%   'keepEmpty'     write frames with no visible target as negatives
%                   (default true)
%   'nodes'         which nodes to export (default all live cameras)
%   'prefix'        filename prefix (default 'frame')
%   'padPixels'     box padding in pixels (default 1), a small margin so
%                   thin structure such as a boom is not clipped
%
% OUTPUT
%   manifest : struct array with one entry per written frame, holding
%              the image path, node, frame index, time, range, and box.
%              Also written to outDir/manifest.csv.
%
% EXAMPLE
%   export_yolo_dataset(scene, 'dataset_swift_cloudy');
%
% See also SIMULATE_UAV_SCENE, SIMULATE_UAV_CAMERAS

p = inputParser;
p.FunctionName = 'export_yolo_dataset';
addRequired(p, 'scene');
addRequired(p, 'outDir');
addParameter(p, 'minPixelArea', 1.5);
addParameter(p, 'keepEmpty', true);
addParameter(p, 'nodes', []);
addParameter(p, 'prefix', 'frame');
addParameter(p, 'padPixels', 1);
parse(p, scene, outDir, varargin{:});
opt = p.Results;

vm = scene.video.meta;
tr = vm.truth;
frames = scene.video.frames;
if ~isnumeric(frames) || isempty(frames)
    error('export_yolo_dataset:noFrames', ...
        ['scene.video.frames is empty. If the scene was saved with ', ...
         '''splitVideo'', reload the companion video file first.']);
end

H = vm.imageSize(1);
W = vm.imageSize(2);
K = numel(vm.frameTimes);
numNodes = size(vm.nodeXY, 1);

if isempty(opt.nodes)
    nodes = find(~vm.deadNodes);
else
    nodes = opt.nodes(:)';
end

imgDir = fullfile(outDir, 'images');
lblDir = fullfile(outDir, 'labels');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end
if ~exist(lblDir, 'dir'), mkdir(lblDir); end

classId = scene.truth.platform.classId;
manifest = struct('image', {}, 'label', {}, 'node', {}, 'frame', {}, ...
    'time', {}, 'range', {}, 'pixelArea', {}, 'bbox', {});

written = 0;
positives = 0;
for n = nodes
    if n < 1 || n > numNodes
        error('export_yolo_dataset:badNode', 'Node index %d out of range.', n);
    end
    for k = 1:K
        hasTarget = tr.inFOV(k, n) && tr.pixelArea(k, n) >= opt.minPixelArea ...
            && all(isfinite(squeeze(tr.bbox(k, n, :))));
        if ~hasTarget && ~opt.keepEmpty
            continue;
        end

        base = sprintf('%s_n%02d_k%04d', opt.prefix, n, k);
        imgPath = fullfile(imgDir, [base, '.png']);
        lblPath = fullfile(lblDir, [base, '.txt']);

        imwrite(frames(:, :, :, k, n), imgPath);

        fid = fopen(lblPath, 'w');
        if fid < 0
            error('export_yolo_dataset:cannotWrite', ...
                'Could not open %s for writing.', lblPath);
        end
        bb = [NaN NaN NaN NaN];
        if hasTarget
            bb = squeeze(tr.bbox(k, n, :))';
            x0 = max(0.5, bb(1) - opt.padPixels);
            y0 = max(0.5, bb(2) - opt.padPixels);
            x1 = min(W + 0.5, bb(1) + bb(3) + opt.padPixels);
            y1 = min(H + 0.5, bb(2) + bb(4) + opt.padPixels);
            xc = ((x0 + x1)/2 - 0.5) / W;
            yc = ((y0 + y1)/2 - 0.5) / H;
            bw = (x1 - x0) / W;
            bh = (y1 - y0) / H;
            fprintf(fid, '%d %.6f %.6f %.6f %.6f\n', classId, xc, yc, bw, bh);
            positives = positives + 1;
        end
        fclose(fid);

        written = written + 1;
        manifest(written).image = imgPath;
        manifest(written).label = lblPath;
        manifest(written).node = n;
        manifest(written).frame = k;
        manifest(written).time = vm.frameTimes(k);
        manifest(written).range = tr.range(k, n);
        manifest(written).pixelArea = tr.pixelArea(k, n);
        manifest(written).bbox = bb;
    end
end

% --- classes file ------------------------------------------------------
fid = fopen(fullfile(outDir, 'classes.txt'), 'w');
if fid >= 0
    names = {'fixedwing', 'multirotor'};
    for c = 0:1
        fprintf(fid, '%s\n', names{c+1});
    end
    fclose(fid);
end

% --- manifest ----------------------------------------------------------
fid = fopen(fullfile(outDir, 'manifest.csv'), 'w');
if fid >= 0
    fprintf(fid, 'image,node,frame,time_s,range_m,pixel_area,u_min,v_min,width,height\n');
    for i = 1:numel(manifest)
        bb = manifest(i).bbox;
        fprintf(fid, '%s,%d,%d,%.4f,%.2f,%.3f,%.2f,%.2f,%.2f,%.2f\n', ...
            manifest(i).image, manifest(i).node, manifest(i).frame, ...
            manifest(i).time, manifest(i).range, manifest(i).pixelArea, ...
            bb(1), bb(2), bb(3), bb(4));
    end
    fclose(fid);
end

fprintf('Exported %d frames (%d with a labeled target) to %s\n', ...
    written, positives, outDir);

end
