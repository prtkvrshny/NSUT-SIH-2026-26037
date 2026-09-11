function map = traversabilityMap(veh, centerline, tracks, params)
% traversabilityMap  Grid classification of the corridor ahead into
% SAFE / UNCERTAIN / BLOCKED cells.
% Implements Phase D items 13-14.
%
%   map = traversabilityMap(veh, centerline, tracks, params)
%
% params: .corridorHalfWidth(4), .horizonDist(30), .sBins(15), .dBins(9),
%   .bufferMargin(1.0)
%
% Output struct `map`:
%   .sGrid, .dGrid - bin center coordinates (sBins x 1, dBins x 1)
%   .state         - sBins x dBins int8 matrix: 0=SAFE,1=UNCERTAIN,2=BLOCKED
%   .stateNames    - {'SAFE','UNCERTAIN','BLOCKED'}
%
% For each (s,d) cell: estimate the time an ego travelling at its
% current speed would reach that longitudinal distance, then check
% every tracked obstacle's predicted position (incl. growing
% uncertainty radius) at that time. BLOCKED if the cell is inside an
% obstacle's uncertainty circle; UNCERTAIN if within a buffer margin
% outside it; SAFE otherwise.
%
% *** Bug-fix #1 relevance ***: this loops over ALL tracks for every
% cell -- there is no notion of "the one obstacle we are worried about".
%
% LIMITATION (see NOTES.md): (s,d) distance is treated as locally
% Euclidean, an approximation that only holds well on roads that are
% not sharply curved.

if nargin < 4, params = struct(); end
halfW        = getOrDefault(params,'corridorHalfWidth',4.0);
horizonDist  = getOrDefault(params,'horizonDist',30);
sBins        = getOrDefault(params,'sBins',15);
dBins        = getOrDefault(params,'dBins',9);
bufferMargin = getOrDefault(params,'bufferMargin',1.0);

[s0, ~] = frenetUtils('cart2frenet', centerline, veh.x, veh.y);
sGrid = linspace(s0, s0+horizonDist, sBins)';
dGrid = linspace(-halfW, halfW, dBins)';

speed = max(veh.v, 1.0);
state = zeros(sBins, dBins, 'int8');

for si = 1:sBins
    tArrive = max((sGrid(si) - s0) / speed, 0);
    for ti = 1:numel(tracks)
        p = predictObstaclePosition(tracks(ti), tArrive);
        [ps, pd] = frenetUtils('cart2frenet', centerline, p.x, p.y);
        for di = 1:dBins
            dist = hypot(sGrid(si)-ps, dGrid(di)-pd);
            if dist <= p.radius
                state(si,di) = 2;
            elseif dist <= p.radius + bufferMargin && state(si,di) < 2
                state(si,di) = 1;
            end
        end
    end
end

map.sGrid = sGrid;
map.dGrid = dGrid;
map.state = state;
map.stateNames = {'SAFE','UNCERTAIN','BLOCKED'};
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
