function varargout = frenetUtils(mode, varargin)
% frenetUtils  Cartesian <-> Frenet (s,d) conversions along a polyline
% centerline. Shared utility used by planning/generateCandidatePaths.m,
% planning/traversabilityMap.m, and simulation/runScenario.m.
%
% Usage:
%   [s, d] = frenetUtils('cart2frenet', centerline, x, y)
%   [x, y, theta] = frenetUtils('frenet2cart', centerline, s, d)
%
% centerline: Nx2 array of [x y] waypoints defining the road REFERENCE
% line. This does NOT imply lanes -- it is only a reference axis; per
% Stage 4 ("no lane-following assumption"), the vehicle and obstacles
% are free to occupy any lateral offset d, positive or negative,
% anywhere across the corridor.
%
%   s = frenetUtils('projectOnPath', xy, sArr, x, y)
%
% *** Bug fix (loop/orbit failure on stale paths) ***: nearest-point
% projection of an arbitrary (x,y) onto an ALREADY-SAMPLED path polyline
% `xy` (e.g. a chosen candidate from generateCandidatePaths.m), returning
% the interpolated arc-length value from the matching `sArr` at that
% projection -- i.e. "where along THIS path is the vehicle actually
% right now", as opposed to cart2frenet's "where along the ROAD
% CENTERLINE is it". Added specifically because
% planning/predictVehiclePosition.m used to assume a path's own first
% sample (`candidateS(1)`) was always the vehicle's current position --
% true only for a path the instant it's generated, false for a
% previously-chosen path being re-validated on a later cycle once the
% vehicle has actually driven partway (or all the way past) it. See the
% comment above predictVehiclePosition.m's call site for the full story,
% and NOTES.md.

switch mode
    case 'cart2frenet'
        centerline = varargin{1}; x = varargin{2}; y = varargin{3};
        [s, d] = cart2frenetImpl(centerline, x, y);
        varargout{1} = s; varargout{2} = d;
    case 'frenet2cart'
        centerline = varargin{1}; s = varargin{2}; d = varargin{3};
        [x, y, theta] = frenet2cartImpl(centerline, s, d);
        varargout{1} = x; varargout{2} = y; varargout{3} = theta;
    case 'projectOnPath'
        xy = varargin{1}; sArr = varargin{2}; x = varargin{3}; y = varargin{4};
        s = projectOnPathImpl(xy, sArr, x, y);
        varargout{1} = s;
    otherwise
        error('frenetUtils:badMode', 'Unknown mode %s', mode);
end
end

function [s, d] = cart2frenetImpl(cl, x, y)
seglen = hypot(diff(cl(:,1)), diff(cl(:,2)));
cumlen = [0; cumsum(seglen)];

bestDist = Inf; bestS = 0; bestD = 0;
for i = 1:size(cl,1)-1
    p1 = cl(i,:); p2 = cl(i+1,:);
    v = p2 - p1; L2 = dot(v,v);
    if L2 < 1e-9, continue; end
    t = dot([x y]-p1, v) / L2;
    t = max(0, min(1, t));
    proj = p1 + t*v;
    dist = hypot(x-proj(1), y-proj(2));
    if dist < bestDist
        bestDist = dist;
        bestS = cumlen(i) + t*seglen(i);
        normal = [-v(2) v(1)] / max(norm(v),1e-9);
        bestD = dot([x y]-proj, normal);
    end
end
s = bestS; d = bestD;
end

function [x, y, theta] = frenet2cartImpl(cl, s, d)
seglen = hypot(diff(cl(:,1)), diff(cl(:,2)));
cumlen = [0; cumsum(seglen)];
s = max(0, min(s, cumlen(end)));

idx = find(cumlen <= s, 1, 'last');
idx = min(idx, size(cl,1)-1);
idx = max(idx, 1);
p1 = cl(idx,:); p2 = cl(idx+1,:);
segL = max(seglen(idx), 1e-9);
t = (s - cumlen(idx)) / segL;
base = p1 + t*(p2-p1);
dirVec = (p2-p1)/segL;
theta = atan2(dirVec(2), dirVec(1));
normal = [-dirVec(2) dirVec(1)];
xy = base + d*normal;
x = xy(1); y = xy(2);
end

function s = projectOnPathImpl(xy, sArr, x, y)
% Nearest-point projection of (x,y) onto the polyline `xy`, whose
% vertices correspond 1:1 to the arc-length values in `sArr` (these are
% NOT recomputed from xy's own Euclidean spacing -- sArr is taken as the
% authoritative road-frame arc length supplied by the caller, matching
% how generateCandidatePaths.m built it). Same segment-projection logic
% as cart2frenetImpl above, generalized to an arbitrary path rather than
% the fixed road centerline.
s = sArr(1); % fallback: degenerate/zero-length path
bestDist = Inf;
for i = 1:size(xy,1)-1
    p1 = xy(i,:); p2 = xy(i+1,:);
    v = p2 - p1; L2 = dot(v,v);
    if L2 < 1e-9, continue; end
    t = dot([x y]-p1, v) / L2;
    t = max(0, min(1, t));
    proj = p1 + t*v;
    dist = hypot(x-proj(1), y-proj(2));
    if dist < bestDist
        bestDist = dist;
        s = sArr(i) + t*(sArr(i+1)-sArr(i));
    end
end
end
