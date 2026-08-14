function install_uav_acoustic_emulator(makePermanent)
%INSTALL_UAV_ACOUSTIC_EMULATOR Add the toolbox folders to the MATLAB path.
%
%   INSTALL_UAV_ACOUSTIC_EMULATOR()
%   INSTALL_UAV_ACOUSTIC_EMULATOR(true) also calls SAVEPATH.

if nargin < 1
    makePermanent = false;
end
if ~(islogical(makePermanent) || isnumeric(makePermanent)) ...
        || ~isscalar(makePermanent) || ~(makePermanent == 0 || makePermanent == 1)
    error('install_uav_acoustic_emulator:InvalidOption', ...
        'makePermanent must be a logical scalar.');
end

rootFolder = fileparts(mfilename('fullpath'));
addpath(rootFolder);
addpath(fullfile(rootFolder, 'examples'));
addpath(fullfile(rootFolder, 'tests'));

if logical(makePermanent)
    status = savepath;
    if status ~= 0
        warning('install_uav_acoustic_emulator:SavePathFailed', ...
            'The path was added for this session, but savepath failed.');
    end
end

fprintf('MATLAB UAV Acoustic Emulator added to the MATLAB path.\n');
end
