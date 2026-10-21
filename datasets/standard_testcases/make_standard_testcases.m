function index = make_standard_testcases(varargin)
%MAKE_STANDARD_TESTCASES Generate the MANTIS-Std standard test-case dataset.
%
%   index = MAKE_STANDARD_TESTCASES() runs MANTIS-Sim for every case in
%   MANTIS_STD_SPECS and writes, into ./MANTIS-Std-v<version>/ next to
%   this file:
%
%     TC01_overhead_pass.mat ... TC06_degraded_pass.mat   one per case
%     previews/TC01_overhead_pass.png ...                quick-look plots
%     index.mat                                          case list + hashes
%     SHA256SUMS.txt                                     file checksums
%
%   Each case file holds six top-level variables (info, params, nodes,
%   audio, video, truth) in MAT v7 format, so it opens in MATLAB with
%   load() and in Python with scipy.io.loadmat. See README.md.
%
%   The run is deterministic: every case has a fixed seed, so the same
%   emulator version produces the same arrays. Content hashes of the
%   main arrays go into index.mat so VERIFY_STANDARD_TESTCASES can
%   confirm a copy was not altered.
%
% NAME-VALUE OPTIONS
%   'outDir'       output folder (default ./MANTIS-Std-v<version> beside
%                  this file)
%   'cases'        subset to build, e.g. {'TC01','TC04'} (default all)
%   'emulatorDir'  MANTIS-Sim emulator folder. Default: auto-detected
%                  at ../../software/v0.1.1/emulator relative to this
%                  file, or already on the path.
%   'previews'     write the PNG quick-look plots (default true)
%   'overwrite'    replace existing case files (default false)
%
% EXAMPLE
%   cd('C:\Users\rbilo\Desktop\Per_MANTIS-Sim\datasets\standard_testcases')
%   make_standard_testcases();
%   verify_standard_testcases();
%
% See also MANTIS_STD_SPECS, MANTIS_STD_BUILD_CASE,
%          VERIFY_STANDARD_TESTCASES

here = fileparts(mfilename('fullpath'));
ver  = mantis_std_version();

p = inputParser;
p.FunctionName = 'make_standard_testcases';
addParameter(p, 'outDir', fullfile(here, ver.folder));
addParameter(p, 'cases', {});
addParameter(p, 'emulatorDir', '');
addParameter(p, 'previews', true);
addParameter(p, 'overwrite', false);
parse(p, varargin{:});
opt = p.Results;

% ---------------------------------------------------------------------
% 1. Emulator on the path
% ---------------------------------------------------------------------
ensure_emulator(here, opt.emulatorDir);

% ---------------------------------------------------------------------
% 2. Which cases
% ---------------------------------------------------------------------
specs = mantis_std_specs();
if ~isempty(opt.cases)
    want = cellstr(opt.cases);
    keep = ismember({specs.id}, want);
    missing = setdiff(want, {specs.id});
    if ~isempty(missing)
        error('make_standard_testcases:unknownCase', ...
            'Unknown case id(s): %s', strjoin(missing, ', '));
    end
    specs = specs(keep);
end

if ~exist(opt.outDir, 'dir'), mkdir(opt.outDir); end
prevDir = fullfile(opt.outDir, 'previews');
if opt.previews && ~exist(prevDir, 'dir'), mkdir(prevDir); end

fprintf('\n%s v%s  ->  %s\n', ver.name, ver.version, opt.outDir);
fprintf('%s\n', repmat('-', 1, 72));

% ---------------------------------------------------------------------
% 3. Build, save, hash, preview
% ---------------------------------------------------------------------
% Merge into any existing index so a partial rebuild keeps the others.
indexFile = fullfile(opt.outDir, 'index.mat');
entries = struct([]);
if exist(indexFile, 'file')
    old = load(indexFile, 'index');
    if isfield(old, 'index') && isfield(old.index, 'cases')
        entries = old.index.cases;
    end
end

for i = 1:numel(specs)
    s = specs(i);
    fileName = sprintf('%s_%s.mat', s.id, s.slug);
    outFile  = fullfile(opt.outDir, fileName);

    if exist(outFile, 'file') && ~opt.overwrite
        fprintf('%-5s %-16s exists, skipped (overwrite=false)\n', s.id, s.slug);
        continue;
    end

    tStart = tic;
    tc = mantis_std_build_case(s);

    % Content hashes over the arrays a tracker actually consumes.
    hashes = struct();
    hashes.audioY      = sha256_of(tc.audio.y);
    hashes.videoFrames = sha256_of(tc.video.frames);
    hashes.truthPos    = sha256_of(tc.truth.pos);
    tc.info.contentSha256 = hashes;

    info   = tc.info;    %#ok<NASGU>
    params = tc.params;  %#ok<NASGU>
    nodes  = tc.nodes;   %#ok<NASGU>
    audio  = tc.audio;   %#ok<NASGU>
    video  = tc.video;   %#ok<NASGU>
    truth  = tc.truth;   %#ok<NASGU>
    % v7 (compressed, HDF5-free) so scipy.io.loadmat can read it.
    save(outFile, 'info', 'params', 'nodes', 'audio', 'video', 'truth', '-v7');

    if opt.previews
        write_preview(fullfile(prevDir, sprintf('%s_%s.png', s.id, s.slug)), tc);
    end

    d = dir(outFile);
    e = struct();
    e.id          = s.id;
    e.slug        = s.slug;
    e.title       = s.title;
    e.file        = fileName;
    e.bytes       = d.bytes;
    e.fileSha256  = file_sha256(outFile);
    e.content     = hashes;
    e.numNodes    = numel(tc.nodes.id);
    e.numSamples  = numel(tc.audio.t);
    e.numFrames   = numel(tc.video.t);
    e.generatedUTC = tc.info.generatedUTC;
    entries = upsert(entries, e);

    alive = tc.nodes.camAlive';
    pa = tc.truth.camera.pixelArea(:, alive);
    fprintf(['%-5s %-16s %6.1f MB  closest %6.1f m  peak target %5.1f px  ', ...
             '(%4.1f s)\n'], s.id, s.slug, d.bytes/2^20, ...
             min(tc.truth.closestRange(tc.nodes.alive)), max([pa(:); 0]), toc(tStart));
end

% ---------------------------------------------------------------------
% 4. Index and checksum list
% ---------------------------------------------------------------------
[~, order] = sort({entries.id});
entries = entries(order);

index = struct();
index.dataset        = ver.name;
index.datasetVersion = ver.version;
index.cases          = entries;
index.specs          = mantis_std_specs();
index.updatedUTC     = utc_now();
save(indexFile, 'index', '-v7');

fid = fopen(fullfile(opt.outDir, 'SHA256SUMS.txt'), 'w');
for i = 1:numel(entries)
    fprintf(fid, '%s  %s\n', entries(i).fileSha256, entries(i).file);
end
fclose(fid);

fprintf('%s\n', repmat('-', 1, 72));
fprintf('%d case(s) in %s. Run verify_standard_testcases() next.\n\n', ...
    numel(entries), opt.outDir);
end

% =====================================================================
function ensure_emulator(here, emulatorDir)
if ~isempty(emulatorDir)
    addpath(emulatorDir);
elseif exist('simulate_uav_scene', 'file') ~= 2
    guess = fullfile(here, '..', '..', 'software', 'v0.1.1', 'emulator');
    if exist(fullfile(guess, 'simulate_uav_scene.m'), 'file')
        addpath(guess);
    end
end
if exist('simulate_uav_scene', 'file') ~= 2
    error('make_standard_testcases:noEmulator', ...
        ['Could not find the MANTIS-Sim emulator. Pass ''emulatorDir'', ', ...
         'e.g. make_standard_testcases(''emulatorDir'', ', ...
         '''C:\\...\\Per_MANTIS-Sim\\software\\v0.1.1\\emulator'').']);
end
end

% =====================================================================
function entries = upsert(entries, e)
if isempty(entries)
    entries = e;
    return;
end
try
    k = find(strcmp({entries.id}, e.id), 1);
    if isempty(k)
        entries(end+1) = e;
    else
        entries(k) = e;
    end
catch
    % Old index from a different schema: start it fresh.
    warning('make_standard_testcases:indexReset', ...
        'Existing index.mat has a different layout; rebuilding it.');
    entries = e;
end
end

% =====================================================================
function h = sha256_of(x)
%SHA256_OF Hash of an array's raw bytes (column-major), via Java.
if islogical(x), x = uint8(x); end
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(x(:), 'uint8'));
h = bytes_to_hex(md.digest());
end

% =====================================================================
function h = file_sha256(file)
fid = fopen(file, 'r');
bytes = fread(fid, inf, '*uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);
h = bytes_to_hex(md.digest());
end

% =====================================================================
function h = bytes_to_hex(d)
d = typecast(int8(d), 'uint8');
h = lower(reshape(dec2hex(d, 2)', 1, []));
end

% =====================================================================
function s = utc_now()
try
    s = char(datetime('now', 'TimeZone', 'UTC'), 'yyyy-MM-dd''T''HH:mm:ss''Z''');
catch
    s = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
end
end

% =====================================================================
function write_preview(pngFile, tc)
%WRITE_PREVIEW One-page quick look: geometry, ranges, loudness, and the
%   six camera views at the frame where the target is most visible.

nodes = tc.nodes;  truth = tc.truth;  audio = tc.audio;  video = tc.video;
N = numel(nodes.id);

ink    = [0.13 0.15 0.20];
muted  = [0.45 0.48 0.55];
accent = [0.85 0.33 0.10];
nodeCol = lines(max(N, 7));

fig = figure('Visible', 'off', 'Color', 'w', 'Units', 'pixels', ...
    'Position', [50 50 1600 980], 'PaperPositionMode', 'auto');

% --- header ------------------------------------------------------------
annotation(fig, 'textbox', [0.04 0.935 0.92 0.05], 'String', ...
    sprintf('%s  |  %s', tc.info.caseId, tc.info.title), ...
    'EdgeColor', 'none', 'Interpreter', 'none', 'FontSize', 18, 'FontWeight', 'bold', 'Color', ink);
annotation(fig, 'textbox', [0.04 0.905 0.92 0.035], 'String', ...
    sprintf('%s v%s  |  %s  |  %g s  |  audio %g Hz  |  %dx%d px @ %g fps  |  sky: %s  |  seed %d', ...
    tc.info.dataset, tc.info.datasetVersion, tc.info.platformName, ...
    tc.params.duration, audio.fs, video.imageSize(1), video.imageSize(2), ...
    video.fps, video.sky, tc.params.seed), ...
    'EdgeColor', 'none', 'Interpreter', 'none', 'FontSize', 10.5, 'Color', muted);

% --- plan view -------------------------------------------------------------
ax1 = axes(fig, 'Position', [0.045 0.50 0.27 0.37]); hold(ax1, 'on');
plot(ax1, truth.pos(:,1), truth.pos(:,2), '-', 'Color', ink, 'LineWidth', 1.6);
plot(ax1, truth.pos(1,1), truth.pos(1,2), 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', ink, 'LineWidth', 1.2);
iMid = round(numel(truth.t) * 0.5);
quiver(ax1, truth.pos(iMid,1), truth.pos(iMid,2), ...
    truth.vel(iMid,1)*2, truth.vel(iMid,2)*2, 0, 'Color', ink, ...
    'LineWidth', 1.4, 'MaxHeadSize', 2);
for n = 1:N
    if nodes.alive(n)
        plot(ax1, nodes.xy(n,1), nodes.xy(n,2), 's', 'MarkerSize', 9, ...
            'MarkerFaceColor', nodeCol(n,:), 'MarkerEdgeColor', 'w');
    else
        plot(ax1, nodes.xy(n,1), nodes.xy(n,2), 'x', 'MarkerSize', 11, ...
            'Color', accent, 'LineWidth', 2.2);
    end
    text(ax1, nodes.xy(n,1)*1.28, nodes.xy(n,2)*1.28, sprintf('%d', n), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', ink);
end
axis(ax1, 'equal'); grid(ax1, 'on'); box(ax1, 'on');
style_axes(ax1, ink, muted);
xlabel(ax1, 'x (m)'); ylabel(ax1, 'y (m)');
title(ax1, sprintf('Plan view  |  altitude %g m', tc.params.altitude), ...
    'FontWeight', 'normal', 'Color', ink, 'Interpreter', 'none');

% --- range to each node ------------------------------------------------------
ax2 = axes(fig, 'Position', [0.375 0.50 0.27 0.37]); hold(ax2, 'on');
for n = 1:N
    ls = '-'; if ~nodes.alive(n), ls = ':'; end
    plot(ax2, truth.t, truth.rangeToNodes(:,n), ls, 'Color', nodeCol(n,:), ...
        'LineWidth', 1.3, 'DisplayName', sprintf('node %d', n));
end
grid(ax2, 'on'); box(ax2, 'on'); style_axes(ax2, ink, muted);
xlabel(ax2, 'time (s)'); ylabel(ax2, 'slant range (m)');
title(ax2, 'Ground-truth range to each node', 'FontWeight', 'normal', 'Color', ink, 'Interpreter', 'none');
legend(ax2, 'Location', 'best', 'Box', 'off', 'FontSize', 8);
xlim(ax2, [truth.t(1) truth.t(end)]);

% --- microphone loudness (RMS in 0.25 s windows) ---------------------------
ax3 = axes(fig, 'Position', [0.705 0.50 0.27 0.37]); hold(ax3, 'on');
win = round(0.25 * audio.fs);
nW  = floor(numel(audio.t) / win);
tW  = (0:nW-1)' * win / audio.fs + win / (2*audio.fs);
for n = 1:N
    if ~nodes.micAlive(n), continue; end   % dead mics are all zeros
    y = double(audio.y(1:nW*win, n));
    r = sqrt(mean(reshape(y, win, nW).^2, 1))';
    plot(ax3, tW, 20*log10(max(r, eps)), '-', 'Color', nodeCol(n,:), 'LineWidth', 1.3);
end
grid(ax3, 'on'); box(ax3, 'on'); style_axes(ax3, ink, muted);
xlabel(ax3, 'time (s)'); ylabel(ax3, 'RMS level (dB, arbitrary ref.)');
ttl = 'Microphone loudness (0.25 s windows)';
if any(~nodes.micAlive)
    ttl = sprintf('%s  |  offline: %s', ttl, mat2str(find(~nodes.micAlive)'));
end
title(ax3, ttl, 'FontWeight', 'normal', 'Color', ink, 'Interpreter', 'none');
xlim(ax3, [truth.t(1) truth.t(end)]);

% --- camera montage at the most visible frame --------------------------------
pa = tc.truth.camera.pixelArea;
pa(:, ~nodes.camAlive) = 0;
[~, kBest] = max(sum(pa, 2));
gapX = 0.012;
w = (0.93 - gapX*(N-1)) / N;
for n = 1:N
    axn = axes(fig, 'Position', [0.045 + (n-1)*(w+gapX), 0.06, w, 0.33]);
    img = video.frames(:,:,kBest,n);
    image(axn, repmat(img, [1 1 3]));
    axis(axn, 'image'); axis(axn, 'off'); hold(axn, 'on');
    bb = squeeze(tc.truth.camera.bbox(kBest, n, :));
    hasBox = nodes.camAlive(n) && tc.truth.camera.inFOV(kBest, n) && ...
        all(isfinite(bb)) && bb(3) > 0 && bb(4) > 0;
    if hasBox
        pad = 4;
        rectangle(axn, 'Position', [bb(1)-pad, bb(2)-pad, bb(3)+2*pad, bb(4)+2*pad], ...
            'EdgeColor', accent, 'LineWidth', 1.4);
    end
    if nodes.camAlive(n)
        lab = sprintf('node %d  |  %.0f m  |  %.1f px', n, ...
            tc.truth.camera.range(kBest, n), tc.truth.camera.pixelArea(kBest, n));
    else
        lab = sprintf('node %d  |  OFFLINE', n);
    end
    title(axn, lab, 'FontWeight', 'normal', 'FontSize', 9, 'Color', ink, 'Interpreter', 'none');
end
annotation(fig, 'textbox', [0.04 0.395 0.92 0.03], 'String', ...
    sprintf(['Fisheye frames at t = %.0f s (frame %d of %d), the frame with the ', ...
    'largest total target area. Box marks the exact rendered target.'], ...
    video.t(kBest), kBest, numel(video.t)), ...
    'EdgeColor', 'none', 'Interpreter', 'none', 'FontSize', 10, 'Color', muted);

% --- save ----------------------------------------------------------------------
try
    exportgraphics(fig, pngFile, 'Resolution', 130);
catch
    print(fig, pngFile, '-dpng', '-r130');
end
close(fig);
end

% =====================================================================
function style_axes(ax, ink, muted)
set(ax, 'FontSize', 9.5, 'XColor', muted, 'YColor', muted, ...
    'GridColor', [0.85 0.86 0.89], 'GridAlpha', 1, 'LineWidth', 0.8);
ax.Title.Color = ink;
end
