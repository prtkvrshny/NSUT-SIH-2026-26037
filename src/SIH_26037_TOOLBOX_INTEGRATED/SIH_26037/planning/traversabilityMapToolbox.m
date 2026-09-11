function map = traversabilityMapToolbox(veh, centerline, tracks, params)
% traversabilityMapToolbox  Navigation Toolbox alternate to
% planning/traversabilityMap.m: SAME SAFE/UNCERTAIN/BLOCKED
% classification (Phase D items 13-14), additionally packaged as a
% real Navigation Toolbox occupancyMap object.
%
% *** TOOLBOX ADD-ON -- see TOOLBOX_INTEGRATION.md ***
% DESIGN CHOICE (read this before the rest): the actual SAFE/UNCERTAIN/
% BLOCKED classification below is NOT reimplemented against the
% toolbox -- it calls the existing, Octave-tested planning/traversabilityMap.m
% directly and reuses its output verbatim. Only an occupancyMap
% wrapper around that already-correct result is toolbox-specific. This
% was a deliberate risk-reduction decision: occupancyMap's exact
% constructor/property API (grid-vs-world coordinate convention, axis
% ordering, origin-offset property name) is genuinely something I
% cannot fully verify from this sandbox, and getting it subtly wrong
% in a from-scratch reimplementation could silently corrupt the
% classification your planner relies on. Reusing the tested function
% for the actual decision, and wrapping its result for toolbox
% interoperability/visualization only, means a mistake in the wrapper
% can only ever cost you the occupancyMap object (map.occMap = [] and
% a warning) -- never a wrong SAFE/BLOCKED call.
%
%   map = traversabilityMapToolbox(veh, centerline, tracks, params)
%
% Output: identical fields to traversabilityMap.m (.sGrid, .dGrid,
% .state, .stateNames), PLUS:
%   .occMap          - Navigation Toolbox occupancyMap object over the
%                      LOCAL (sLocal, dLocal) grid (sLocal = s-s0 in
%                      [0,horizonDist], dLocal = d+halfWidth in
%                      [0,2*halfWidth]) if construction succeeded,
%                      otherwise [].
%   .occMapWarning   - '' if occMap built successfully, else a message
%                      explaining what went wrong (e.g. toolbox not
%                      licensed, or an API mismatch this code didn't
%                      anticipate).
%
% CONFIDENCE: the classification (.state/.sGrid/.dGrid) is exactly as
% trustworthy as planning/traversabilityMap.m already is (Octave-tested
% -- see NOTES.md). The occupancyMap wrapper itself is UNTESTED (no
% Navigation Toolbox in this sandbox's Octave); the try/catch below
% means a wrong API guess degrades to occMap=[] rather than corrupting
% anything a caller relies on for actual decisions.

baseMap = traversabilityMap(veh, centerline, tracks, params);
map = baseMap;
map.occMap = [];
map.occMapWarning = '';

try
    sBins = numel(baseMap.sGrid);
    dBins = numel(baseMap.dGrid);
    if sBins < 2 || dBins < 2
        error('traversabilityMapToolbox:tooFewBins', ...
              'Need >=2 bins per axis to derive a grid resolution.');
    end
    sRes = (baseMap.sGrid(end) - baseMap.sGrid(1)) / (sBins-1);
    dRes = (baseMap.dGrid(end) - baseMap.dGrid(1)) / (dBins-1);
    cellSize = mean([sRes dRes]); % occupancyMap uses one resolution for both axes
    resolution = 1/max(cellSize, 1e-3); % cells per metre

    % Occupancy probability: 0=SAFE, 0.5=UNCERTAIN, 1=BLOCKED. This
    % project's own .state matrix already IS 0/1/2 -- just rescale.
    occGrid = double(baseMap.state) / 2;

    % occupancyMap(matrix, resolution) builds a map whose grid directly
    % mirrors the input matrix at the given cells/metre resolution,
    % with the world origin at the matrix's (1,1) corner by default --
    % this is the local (sLocal=0, dLocal=0) corner in this project's
    % convention, so no origin/offset property needs to be set at all
    % (that property's exact name is the single detail about
    % occupancyMap I am least sure of -- sidestepped entirely here by
    % construction, not by getting it right).
    occMap = occupancyMap(occGrid, resolution);

    map.occMap = occMap;
catch ME
    map.occMapWarning = sprintf(['occupancyMap construction failed (%s: %s) -- ' ...
        'falling back to the plain classification only. This does not ' ...
        'affect .state/.sGrid/.dGrid, which come from the already-tested ' ...
        'planning/traversabilityMap.m regardless.'], ME.identifier, ME.message);
end
end
