function [img, truth] = render_uav_projection(img, cam, plat, pos, R, opts)
%RENDER_UAV_PROJECTION Draw a UAV into a fisheye image and report truth.
%
%   [img, truth] = RENDER_UAV_PROJECTION(img, cam, plat, pos, R, opts)
%   composites the platform silhouette onto the background image img
%   (H-by-W-by-C, normalized radiance) and returns per-frame ground
%   truth for the target.
%
% INPUTS
%   img  : background image to draw into, H-by-W-by-C in [0,1]
%   cam  : camera struct from MAKE_FISHEYE_CAMERA
%   plat : platform struct from UAV_PLATFORM
%   pos  : 1-by-3 world position of the UAV body origin (m)
%   R    : 3-by-3 body-to-world rotation matrix
%   opts : struct, all fields optional
%       .superSample   subpixel samples per axis for coverage
%                      (default 3). The target is often only a few
%                      pixels across, so fractional coverage matters
%                      more than it would for a large object.
%       .edgeDivisions number of segments each polygon edge is split
%                      into before projection (default 4). This is what
%                      makes long straight members bend correctly
%                      through the fisheye instead of being drawn as
%                      chords.
%       .navLights     true/false, draw nav and strobe lights
%       .time          current time (s), used for strobe phase
%       .lightGain     scale on light intensity (default 1)
%
% OUTPUT truth (struct)
%   .inFOV       true if the body origin lies inside the field of view
%   .u, .v       pixel coordinates of the body origin
%   .range       slant range from camera to body origin (m)
%   .bbox        [uMin vMin width height] in pixels, clipped to the
%                image, NaN if nothing was drawn
%   .pixelArea   total composited coverage in pixels (a direct measure
%                of how much signal a detector has to work with)
%   .angularSize apparent angular size of the platform (deg)
%
% See also FISHEYE_PROJECT, UAV_PLATFORM, SIMULATE_UAV_CAMERAS

if nargin < 6, opts = struct(); end
if ~isfield(opts, 'superSample'),   opts.superSample = 3;   end
if ~isfield(opts, 'edgeDivisions'), opts.edgeDivisions = 4; end
if ~isfield(opts, 'navLights'),     opts.navLights = false; end
if ~isfield(opts, 'time'),          opts.time = 0;          end
if ~isfield(opts, 'lightGain'),     opts.lightGain = 1;     end

H = size(img, 1);
W = size(img, 2);
C = size(img, 3);
S = max(1, round(opts.superSample));

% --- body origin: range, angular size, visibility ---------------------
[u0, v0, ~, ~, inFOV0, range0] = fisheye_project(pos(:)', cam);
truth = struct('inFOV', inFOV0, 'u', u0, 'v', v0, 'range', range0, ...
    'bbox', [NaN NaN NaN NaN], 'pixelArea', 0, ...
    'angularSize', 2*atan2(plat.sizeRef/2, range0) * 180/pi);

if ~inFOV0
    return;
end

% --- project every (densified) face -----------------------------------
uAll = [];
vAll = [];
totalCoverage = 0;

for k = 1:numel(plat.faces)
    idx = plat.faces(k).idx;
    Pbody = plat.verts(idx, :);
    Pbody = densify_polygon(Pbody, opts.edgeDivisions);
    Pworld = Pbody * R.' + repmat(pos(:)', size(Pbody, 1), 1);

    [uu, vv, ~, ~, inFOV] = fisheye_project(Pworld, cam);
    if ~all(inFOV)
        continue;   % face crosses the FOV rim; skip it at this fidelity
    end

    uAll = [uAll; uu]; %#ok<AGROW>
    vAll = [vAll; vv]; %#ok<AGROW>

    % Bounding box in pixels, padded so partial edge pixels are caught.
    c0 = max(1, floor(min(uu)) - 1);
    c1 = min(W, ceil(max(uu)) + 1);
    r0 = max(1, floor(min(vv)) - 1);
    r1 = min(H, ceil(max(vv)) + 1);
    if c1 < c0 || r1 < r0
        continue;
    end

    % Fractional coverage by supersampling inside the bounding box.
    cols = c0:c1;
    rows = r0:r1;
    nc = numel(cols);
    nr = numel(rows);
    offs = ((1:S) - 0.5)/S - 0.5;
    [subC, subR] = meshgrid(offs, offs);

    % One inpolygon call over every subsample of every pixel in the
    % bounding box: far cheaper than testing each subsample offset
    % separately, and the boxes are small.
    [Cg, Rg] = meshgrid(cols, rows);
    Cg = Cg(:); Rg = Rg(:);
    nPix = numel(Cg);
    nSubPix = S*S;
    Xq = repmat(Cg, 1, nSubPix) + repmat(subC(:)', nPix, 1);
    Yq = repmat(Rg, 1, nSubPix) + repmat(subR(:)', nPix, 1);
    inMask = inpolygon(Xq(:), Yq(:), uu, vv);
    cov = reshape(mean(reshape(double(inMask), nPix, nSubPix), 2), nr, nc);
    if ~any(cov(:) > 0)
        continue;
    end

    alpha = plat.faces(k).alpha * cov;
    gray = plat.faces(k).gray;
    for c = 1:C
        patch = img(rows, cols, c);
        img(rows, cols, c) = patch .* (1 - alpha) + gray * alpha;
    end
    totalCoverage = totalCoverage + sum(cov(:)) * plat.faces(k).alpha;
end

% --- nav and strobe lights -------------------------------------------
if opts.navLights && isfield(plat, 'lights') && ~isempty(plat.lights.pos)
    Lw = plat.lights.pos * R.' + repmat(pos(:)', size(plat.lights.pos,1), 1);
    [ul, vl, ~, ~, inFOVl] = fisheye_project(Lw, cam);
    for m = 1:size(Lw, 1)
        if ~inFOVl(m), continue; end
        on = true;
        if plat.lights.strobeHz(m) > 0
            ph = mod(opts.time * plat.lights.strobeHz(m), 1);
            on = ph < plat.lights.duty(m);
        end
        if ~on, continue; end
        amp = opts.lightGain * plat.lights.intensity(m);
        img = add_point_light(img, ul(m), vl(m), amp, plat.lights.rgb(m,:));
    end
end

% --- pack truth -------------------------------------------------------
if ~isempty(uAll)
    uMin = max(1, min(uAll)); uMax = min(W, max(uAll));
    vMin = max(1, min(vAll)); vMax = min(H, max(vAll));
    if uMax > uMin && vMax > vMin
        truth.bbox = [uMin, vMin, uMax - uMin, vMax - vMin];
    else
        truth.bbox = [uMin, vMin, 1, 1];
    end
end
truth.pixelArea = totalCoverage;

end

% =====================================================================
function Q = densify_polygon(P, nDiv)
%DENSIFY_POLYGON Insert nDiv-1 extra points along each polygon edge so
%   that straight structural members project as the curves the fisheye
%   actually produces.
if nDiv <= 1
    Q = P;
    return;
end
n = size(P, 1);
Q = zeros(n*nDiv, 3);
w = ((0:nDiv-1)') / nDiv;
for i = 1:n
    j = mod(i, n) + 1;
    seg = repmat(P(i,:), nDiv, 1) + w * (P(j,:) - P(i,:));
    Q((i-1)*nDiv + (1:nDiv), :) = seg;
end
end

% =====================================================================
function img = add_point_light(img, u, v, amp, rgb)
%ADD_POINT_LIGHT Additive Gaussian blob for a nav or strobe light.
H = size(img, 1); W = size(img, 2); C = size(img, 3);
sig = 0.8;
c0 = max(1, floor(u - 3)); c1 = min(W, ceil(u + 3));
r0 = max(1, floor(v - 3)); r1 = min(H, ceil(v + 3));
if c1 < c0 || r1 < r0, return; end
[Cg, Rg] = meshgrid(c0:c1, r0:r1);
blob = amp * exp(-((Cg - u).^2 + (Rg - v).^2) / (2*sig^2));
for c = 1:C
    if C == 3
        w = rgb(c);
    else
        w = mean(rgb);
    end
    img(r0:r1, c0:c1, c) = img(r0:r1, c0:c1, c) + w * blob;
end
end
