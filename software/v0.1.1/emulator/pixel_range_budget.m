function budget = pixel_range_budget(platform, varargin)
%PIXEL_RANGE_BUDGET Relate camera resolution to how far a UAV stays visible.
%
%   budget = PIXEL_RANGE_BUDGET(platform) reports, for a given camera
%   configuration, the slant range at which the platform spans a chosen
%   number of pixels. This is the sizing question the emulator exists to
%   answer: a wide fisheye spreads the whole sky over a small sensor, so
%   angular resolution, not signal level, is usually what limits how far
%   away a target can be detected in daylight.
%
% INPUTS
%   platform : name ('swift', 'f550') or a struct from UAV_PLATFORM
%
% NAME-VALUE OPTIONS
%   'imageSize'    [rows cols], scalar allowed (default [240 240])
%   'fovDeg'       full fisheye field of view (default 185)
%   'projection'   'equidistant' (default) | 'equisolid' | 'orthographic'
%   'pixelTargets' target extents to report (default [1 3 5 10 20])
%   'ranges'       ranges in meters at which to report pixel extent
%                  (default [10 25 50 100 200])
%   'print'        print a formatted table (default true)
%
% OUTPUT budget
%   .pixPerRad, .degPerPixel, .mradPerPixel
%   .pixelTargets, .rangeForPixels    range (m) at each target extent
%   .ranges, .pixelsAtRange           extent (px) at each range
%
% CAVEAT
%   The extent reported is the full span of the airframe. A detector
%   needs considerably more than one pixel of extent to work: roughly
%   3 to 5 px for a matched-filter or track-before-detect approach on a
%   point-like target, and more like 10 to 20 px before a learned
%   detector such as YOLO becomes reliable.
%
% EXAMPLE
%   pixel_range_budget('swift', 'imageSize', [240 480 960]);
%
% See also MAKE_FISHEYE_CAMERA, SIMULATE_UAV_CAMERAS

p = inputParser;
p.FunctionName = 'pixel_range_budget';
addRequired(p, 'platform');
addParameter(p, 'imageSize', [240 240]);
addParameter(p, 'fovDeg', 185);
addParameter(p, 'projection', 'equidistant');
addParameter(p, 'pixelTargets', [1 3 5 10 20]);
addParameter(p, 'ranges', [10 25 50 100 200]);
addParameter(p, 'print', true);
parse(p, platform, varargin{:});
opt = p.Results;

if isstruct(platform)
    plat = platform;
else
    plat = uav_platform(platform);
end

sz = opt.imageSize;
if isscalar(sz), sz = [sz sz]; end
if numel(sz) > 2
    % A list of square sizes was supplied: recurse and report each.
    budget = struct([]);
    for i = 1:numel(sz)
        b = pixel_range_budget(plat, 'imageSize', [sz(i) sz(i)], ...
            'fovDeg', opt.fovDeg, 'projection', opt.projection, ...
            'pixelTargets', opt.pixelTargets, 'ranges', opt.ranges, ...
            'print', opt.print);
        if isempty(budget), budget = b; else, budget(end+1) = b; end %#ok<AGROW>
    end
    return;
end

cam = make_fisheye_camera([0 0 0], 'imageSize', sz, ...
    'fovDeg', opt.fovDeg, 'projection', opt.projection);

radPerPix = 1 / cam.pixPerRad;
D = plat.sizeRef;

budget = struct();
budget.platform = plat.name;
budget.imageSize = sz;
budget.fovDeg = opt.fovDeg;
budget.pixPerRad = cam.pixPerRad;
budget.degPerPixel = radPerPix * 180/pi;
budget.mradPerPixel = radPerPix * 1000;
budget.pixelTargets = opt.pixelTargets(:)';
budget.rangeForPixels = D ./ (2*tan(opt.pixelTargets(:)' * radPerPix / 2));
budget.ranges = opt.ranges(:)';
budget.pixelsAtRange = 2*atan2(D/2, opt.ranges(:)') / radPerPix;

if opt.print
    fprintf('\n%s, %.1f m across\n', plat.name, D);
    fprintf('camera: %dx%d px, %.0f deg FOV, %s -> %.3f deg (%.2f mrad) per pixel\n', ...
        sz(1), sz(2), opt.fovDeg, opt.projection, ...
        budget.degPerPixel, budget.mradPerPixel);
    fprintf('  target extent   range\n');
    for i = 1:numel(budget.pixelTargets)
        fprintf('  %5.0f px       %7.1f m\n', ...
            budget.pixelTargets(i), budget.rangeForPixels(i));
    end
    fprintf('  range           target extent\n');
    for i = 1:numel(budget.ranges)
        fprintf('  %5.0f m        %7.2f px\n', ...
            budget.ranges(i), budget.pixelsAtRange(i));
    end
    fprintf('\n');
end

end
