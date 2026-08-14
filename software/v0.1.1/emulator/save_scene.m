function outFile = save_scene(filename, scene, varargin)
%SAVE_SCENE Save a joint acoustic and visual scene to a MAT file.
%
%   outFile = SAVE_SCENE(filename, scene) writes the whole scene struct
%   (audio, video frames, metadata, ground truth) to a v7.3 MAT file.
%
%   outFile = SAVE_SCENE(filename, scene, 'splitVideo', true) writes the
%   audio and metadata to filename and the frame array to a companion
%   file named <filename>_video.mat. This keeps the main file small
%   enough to load quickly when only the audio and ground truth are
%   needed, which is the common case while developing detectors.
%
% NAME-VALUE OPTIONS
%   'splitVideo'   write frames to a separate file (default false)
%   'dropExtras'   omit scene.audio.extras, the diagnostic clean signal,
%                  distances, delays, and Doppler estimate (default
%                  false). These roughly double the audio file size.
%
% OUTPUT
%   outFile : the path actually written
%
% EXAMPLE
%   save_scene('run_swift_day.mat', scene, 'splitVideo', true);
%
% See also SIMULATE_UAV_SCENE, SAVE_SIMULATION, EXPORT_YOLO_DATASET

p = inputParser;
p.FunctionName = 'save_scene';
addRequired(p, 'filename');
addRequired(p, 'scene');
addParameter(p, 'splitVideo', false);
addParameter(p, 'dropExtras', false);
parse(p, filename, scene, varargin{:});
opt = p.Results;

[~, ~, ext] = fileparts(filename);
if isempty(ext)
    filename = [filename, '.mat'];
end
outFile = filename;

if opt.dropExtras && isfield(scene, 'audio') && isfield(scene.audio, 'extras')
    scene.audio = rmfield(scene.audio, 'extras');
end

if opt.splitVideo
    [pathStr, baseName, ~] = fileparts(filename);
    videoFile = fullfile(pathStr, [baseName, '_video.mat']);
    frames = scene.video.frames; %#ok<NASGU>
    videoMeta = scene.video.meta; %#ok<NASGU>
    save_compat(videoFile, {'frames', 'videoMeta'}, {frames, videoMeta});
    scene.video = rmfield(scene.video, 'frames');
    scene.video.framesFile = videoFile;
    fprintf('Video saved to: %s\n', videoFile);
end

save_compat(outFile, {'scene'}, {scene});
fprintf('Scene saved to: %s\n', outFile);

end

% =====================================================================
function save_compat(file, names, values)
%SAVE_COMPAT Save named variables, preferring the v7.3 format for the
%   large arrays and falling back gracefully where it is unsupported.
for k = 1:numel(names)
    eval([names{k}, ' = values{k};']);
end
try
    save(file, names{:}, '-v7.3');
catch
    save(file, names{:});
end
end
