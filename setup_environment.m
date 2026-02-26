function setup_environment()
%SETUP_ENVIRONMENT Configure MATLAB path and Simulink build directories.
%
%   SETUP_ENVIRONMENT adds all project subfolders to the MATLAB path and
%   redirects Simulink code-generation and cache artefacts to the local
%   build/ directory, keeping generated files out of the source tree.
%
%   Usage:
%       run('setup_environment.m')   % from the repository root, or
%       setup_environment()          % if the root is already on the path
%
%   The script is idempotent: running it multiple times is safe.
%
%   See also: Simulink.fileGenControl, addpath, genpath.

% -------------------------------------------------------------------------
% 1. Locate repository root (directory that contains this script)
% -------------------------------------------------------------------------
scriptDir = fileparts(mfilename('fullpath'));

% -------------------------------------------------------------------------
% 2. Add project subfolders to the MATLAB path
% -------------------------------------------------------------------------
projectFolders = { ...
    'data', ...
    'models', ...
    'controllers', ...
    'scenarios', ...
    'validation', ...
    'utils', ...
    'tests' ...
};

fprintf('=== Vehicle Dynamics MBD — Environment Setup ===\n');
for k = 1:numel(projectFolders)
    folderPath = fullfile(scriptDir, projectFolders{k});
    if isfolder(folderPath)
        addpath(genpath(folderPath));
        fprintf('  [+] Added to path: %s\n', folderPath);
    else
        fprintf('  [!] Folder not found (skipped): %s\n', folderPath);
    end
end

% -------------------------------------------------------------------------
% 3. Configure Simulink code-generation and cache directories
%    All generated artefacts go into build/ which is git-ignored.
% -------------------------------------------------------------------------
buildDir   = fullfile(scriptDir, 'build');
cacheDir   = fullfile(buildDir,  'cache');
codegenDir = fullfile(buildDir,  'codegen');

% Create directories if they do not yet exist
for d = {buildDir, cacheDir, codegenDir}
    if ~isfolder(d{1})
        mkdir(d{1});
        fprintf('  [+] Created directory: %s\n', d{1});
    end
end

% Apply Simulink file-generation control settings
Simulink.fileGenControl('set', ...
    'CacheFolder',        cacheDir, ...
    'CodeGenFolder',      codegenDir, ...
    'createDir',          true);

fprintf('  [+] Simulink CacheFolder   -> %s\n', cacheDir);
fprintf('  [+] Simulink CodeGenFolder -> %s\n', codegenDir);

% -------------------------------------------------------------------------
% 4. Done
% -------------------------------------------------------------------------
fprintf('=== Setup complete. You can now run main_simulation.m ===\n');

end
