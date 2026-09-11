function tracks = smoothTrackTypes(tracks, params)
% smoothTrackTypes  Majority-vote hysteresis over each track's recent
% type labels, to fix the "camera classification flicker" limitation
% flagged in NOTES.md ("a single track's inferred .type can visibly
% flip between calls... A real system would use majority-vote/
% hysteresis over several frames").
%
% *** WHY THIS IS HERE INSTEAD OF A DEEP LEARNING TOOLBOX MODEL ***
% Deep Learning Toolbox was deliberately NOT used for camera
% classification in this delivery. This simulation's virtualCamera.m
% never produces real images -- it produces a ground-truth type label
% plus a flat misclassification probability. There is no real image
% data anywhere in this project to train a network on; a "trained"
% classifier here would necessarily mean hand-fabricated network
% weights standing in for training that never happened, which I'm not
% willing to present as a real model -- it would look like toolbox
% usage without being genuine capability, and I'd have no way to
% verify it does anything sensible. This function fixes the ACTUAL
% documented problem (flicker) with a real, verifiable, Octave-tested
% technique instead. Deep Learning Toolbox is the right tool once
% actual camera IMAGES (not simulated labels) exist to train on --
% see TOOLBOX_INTEGRATION.md.
%
%   tracks = smoothTrackTypes(tracks, params)
%
% Call this AFTER trackObstacles.m/trackObstaclesToolbox.m, once per
% planning cycle, on the full current track list. Maintains a small
% rolling history of each track id's raw .type across calls (a plain
% MATLAB containers.Map, not a struct field, so it works unmodified
% regardless of which tracker produced `tracks`) and overwrites
% tracks(i).type with the majority vote over the last `windowSize`
% raw observations for that id.
%
% params: .windowSize (default 5), .resetHistory (default false -- set
%   true on the FIRST call of a new scenario run so leftover history
%   from a previous run's track IDs can't leak in; simulation/runScenario.m
%   does this for you when opts.smoothTypes=true).
%
% NOT called by default -- see simulation/runScenario.m's `perceive`
% subfunction, gated behind opts.smoothTypes (default false), so the
% tested default pipeline's behavior/metrics are completely unchanged
% unless a caller explicitly opts in.

if nargin < 2, params = struct(); end
windowSize = getOrDefault(params,'windowSize',5);
resetHistory = isfield(params,'resetHistory') && params.resetHistory;

persistent history % containers.Map: track id -> cell array of recent raw types
if isempty(history) || resetHistory
    history = containers.Map('KeyType','double','ValueType','any');
end

for i = 1:numel(tracks)
    id = tracks(i).id;
    rawType = tracks(i).type;

    if isKey(history, id)
        h = history(id);
    else
        h = {};
    end
    h{end+1} = rawType;
    if numel(h) > windowSize
        h = h(end-windowSize+1:end);
    end
    history(id) = h;

    tracks(i).type = majorityVote(h);
end

% Drop history for ids no longer present, so the map doesn't grow
% unbounded over a long run.
if ~isempty(history)
    liveIds = [tracks.id];
    allKeys = cell2mat(keys(history));
    for k = 1:numel(allKeys)
        if ~ismember(allKeys(k), liveIds)
            remove(history, allKeys(k));
        end
    end
end
end

function v = majorityVote(cellOfStrings)
uniqueVals = unique(cellOfStrings);
counts = zeros(1,numel(uniqueVals));
for i = 1:numel(uniqueVals)
    counts(i) = sum(strcmp(cellOfStrings, uniqueVals{i}));
end
[~, bestIdx] = max(counts);
v = uniqueVals{bestIdx};
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
