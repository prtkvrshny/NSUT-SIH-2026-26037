function runAllAnimations(opts)
% runAllAnimations  Run the live dashboard-style animation (see
% visualization/animateScenario.m) for all 5 environments back to back,
% optionally saving each as an .mp4 -- intended as a RoadRunner
% substitute for demos/pitches if a real 3D RoadRunner integration isn't
% ready in time.
%
%   runAllAnimations                          % live view only, no save
%   runAllAnimations(struct('saveVideo', true))  % also save .mp4 files
%
% opts is passed straight through to animateScenario.m for every
% environment (see that file's header for all fields) -- most commonly
% you'll want:
%   .saveVideo  - true/false (default false)
%   .showLive   - true/false (default true)
%   .sensorMode - default 'fusion'; pass 'groundtruth' for a
%                 glitch-free demo reel that skips simulated sensor
%                 noise (see NOTES.md and animateScenario.m's header for
%                 why 'fusion' can occasionally show noisy "ghost"
%                 detections at long range)
%
% Videos, if saved, land in results/animations/<envName>.mp4

if nargin < 1, opts = struct(); end
setupProject();
addpath(fullfile(fileparts(mfilename('fullpath')), 'visualization'));

envNames = {'village','intersection','highway','market','cattle'};
for e = 1:numel(envNames)
    fprintf('\n=== Animating: %s ===\n', envNames{e});
    animateScenario(envNames{e}, opts);
end
end
