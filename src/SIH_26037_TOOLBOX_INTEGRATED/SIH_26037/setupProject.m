function setupProject()
% setupProject  Add all project subfolders to the MATLAB/Octave path.
% Run once per session before calling any runStage*.m script or runDemo.m.

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'vehicle'));
addpath(fullfile(root,'sensors'));
addpath(fullfile(root,'planning'));
addpath(fullfile(root,'simulation'));
% visualization/ and tools/ were previously left off this list (you had
% to addpath them by hand for animateScenario.m/runAllAnimations.m) --
% added here as a small, unrelated convenience fix while touching this
% file for the toolbox add-ons; see TOOLBOX_INTEGRATION.md.
addpath(fullfile(root,'visualization'));
addpath(fullfile(root,'tools'));
fprintf('SIH 26037 project paths added.\n');
end
