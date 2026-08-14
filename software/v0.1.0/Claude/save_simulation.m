function outFile = save_simulation(filename, Y, t, meta, extras)
%SAVE_SIMULATION Save simulation outputs, metadata, and settings to a MAT file.
%
%   outFile = SAVE_SIMULATION(filename, Y, t, meta) saves the signal
%   matrix, time vector, and metadata struct to filename (a .mat file
%   is created if no extension is given).
%
%   outFile = SAVE_SIMULATION(filename, Y, t, meta, extras) additionally
%   stores the optional extras struct (cleanY, distances, delays,
%   doppler).
%
% INPUTS
%   filename : path/name to save to. '.mat' is appended if missing.
%   Y        : numSamples-by-numMics simulated (noisy) signal matrix
%   t        : time vector
%   meta     : metadata struct, as returned by simulate_uav_acoustics
%   extras   : (optional) struct with diagnostic outputs
%
% OUTPUT
%   outFile : the full path actually written to disk
%
% EXAMPLE
%   [Y, t, meta, extras] = simulate_uav_acoustics(micXY, uavXYZ, tVec);
%   save_simulation('run_001.mat', Y, t, meta, extras);

if nargin < 5
    extras = struct();
end

[~, ~, ext] = fileparts(filename);
if isempty(ext)
    filename = [filename, '.mat'];
end

outFile = filename;

% Use a -v7.3 file so large signal matrices (e.g. many mics x long
% duration) save reliably; falls back gracefully on older MATLAB/Octave
% if -v7.3 is unsupported.
try
    save(outFile, 'Y', 't', 'meta', 'extras', '-v7.3');
catch
    save(outFile, 'Y', 't', 'meta', 'extras');
end

fprintf('Simulation saved to: %s\n', outFile);

end
