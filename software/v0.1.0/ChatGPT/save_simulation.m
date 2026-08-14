function savedPath = save_simulation(filename, Y, t, meta, cleanY, distances, delays, doppler)
%SAVE_SIMULATION Save simulator outputs and metadata to a MAT file.
%
%   savedPath = SAVE_SIMULATION(filename, Y, t, meta)
%   savedPath = SAVE_SIMULATION(filename, Y, t, meta, cleanY, ...
%       distances, delays, doppler)
%
% The MAT file contains variables named Y, t, meta, cleanY, distances,
% delays, and doppler. Optional values that are not supplied are saved as
% empty arrays. Large payloads automatically use MAT-file version 7.3.

if nargin < 4
    error('save_simulation:NotEnoughInputs', ...
        'filename, Y, t, and meta are required.');
end
if nargin < 5
    cleanY = [];
end
if nargin < 6
    distances = [];
end
if nargin < 7
    delays = [];
end
if nargin < 8
    doppler = [];
end

filename = local_to_char(filename);
if isempty(strtrim(filename))
    error('save_simulation:InvalidFilename', ...
        'filename must not be empty.');
end
[folder, baseName, extension] = fileparts(filename);
if isempty(extension)
    extension = '.mat';
end
if isempty(baseName)
    error('save_simulation:InvalidFilename', ...
        'filename must include a file name.');
end
if isempty(folder)
    folder = pwd;
elseif ~exist(folder, 'dir')
    [madeFolder, message] = mkdir(folder);
    if ~madeFolder
        error('save_simulation:CannotCreateFolder', ...
            'Could not create output folder: %s', message);
    end
end
savedPath = fullfile(folder, [baseName extension]);

if ~isnumeric(Y) || ~isreal(Y) || ndims(Y) ~= 2 || isempty(Y)
    error('save_simulation:InvalidSignal', ...
        'Y must be a nonempty real numeric matrix.');
end
if ~isnumeric(t) || ~isreal(t) || ~isvector(t) || numel(t) ~= size(Y, 1) ...
        || any(~isfinite(t(:))) || any(diff(t(:)) <= 0)
    error('save_simulation:InvalidTimeVector', ...
        't must be a finite increasing vector with one value per row of Y.');
end
if ~isstruct(meta) || ~isscalar(meta)
    error('save_simulation:InvalidMetadata', ...
        'meta must be a scalar structure.');
end

payload = struct();
payload.Y = Y;
payload.t = t(:);
payload.meta = meta;
payload.cleanY = cleanY;
payload.distances = distances;
payload.delays = delays;
payload.doppler = doppler;

payloadInfo = whos('payload');
if payloadInfo.bytes > 1.8e9
    save(savedPath, '-struct', 'payload', '-v7.3');
else
    save(savedPath, '-struct', 'payload', '-v7');
end
end

function text = local_to_char(value)
if isstring(value)
    if ~isscalar(value)
        error('save_simulation:InvalidFilename', ...
            'filename must be scalar text.');
    end
    text = char(value);
elseif ischar(value)
    text = value;
else
    error('save_simulation:InvalidFilename', ...
        'filename must be a character vector or scalar string.');
end
end
