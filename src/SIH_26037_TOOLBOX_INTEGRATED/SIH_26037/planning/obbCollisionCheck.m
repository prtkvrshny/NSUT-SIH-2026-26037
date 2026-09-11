function [collides, gap] = obbCollisionCheck(poseA, dimsA, poseB, dimsB)
% obbCollisionCheck  Oriented-bounding-box overlap test between two
% rectangular footprints (Separating Axis Theorem).
%
% *** TOOLBOX ADD-ON (see TOOLBOX_INTEGRATION.md) ***
% Written to consume vehicle/obstacleFootprints.m's per-type
% vehicleDimensions-backed sizes (a real per-type footprint, replacing
% the flat 1.5 m circle radius flagged as a known limitation in
% NOTES.md). The geometry itself is deliberately plain MATLAB --
% base-MATLAB math, no toolbox call -- so it is FULLY Octave-testable,
% unlike most of the other toolbox add-ons in this delivery. See
% TOOLBOX_INTEGRATION.md for the actual test cases run.
%
%   [collides, gap] = obbCollisionCheck(poseA, dimsA, poseB, dimsB)
%
% poseA, poseB : [x y theta] -- centre position [m] and heading [rad]
% dimsA, dimsB : struct/object with .Length, .Width [m] (e.g. the
%                output of vehicle/obstacleFootprints.m, or a real
%                vehicleDimensions object -- both expose these fields)
%
% Output:
%   collides - true if the two oriented rectangles overlap
%   gap      - if NOT colliding, the largest per-axis separation found
%              across the 4 candidate axes (a lower-bound estimate of
%              true separation, not the exact minimum distance between
%              the two rectangles -- for two separated rotated
%              rectangles the true nearest-corner distance can
%              legitimately be larger than this axis-projection gap;
%              treat `gap` as "at least this far apart", not as an
%              exact range). If colliding, gap is 0.
%
% Method: classic 2D SAT for two convex polygons -- for two rectangles
% there are only 4 unique candidate separating axes (each rectangle's
% own two edge-normal directions, since opposite edges are parallel).
% The boxes overlap iff their projections onto EVERY one of those 4
% axes overlap; if any single axis shows a gap, the boxes are
% separated and that axis is a valid separating axis. This boolean
% collide/no-collide result is standard and exact for two rectangles;
% only the returned `gap` distance (used for diagnostics only, not for
% the collide decision) is an approximation -- see note above.

cornersA = boxCorners(poseA, dimsA);
cornersB = boxCorners(poseB, dimsB);

axes4 = [edgeNormal(poseA(3)); edgeNormal(poseA(3)+pi/2); ...
         edgeNormal(poseB(3)); edgeNormal(poseB(3)+pi/2)];

collides = true;
maxGap = -Inf;

for k = 1:size(axes4,1)
    ax = axes4(k,:);
    [minA, maxA] = projectOntoAxis(cornersA, ax);
    [minB, maxB] = projectOntoAxis(cornersB, ax);

    % Gap along this axis: positive means separated on this axis.
    thisGap = max(minA, minB) - min(maxA, maxB);
    if thisGap > 0
        collides = false;
    end
    maxGap = max(maxGap, thisGap);
end

if collides
    gap = 0;
else
    gap = max(maxGap, 0);
end
end

function corners = boxCorners(pose, dims)
% 4x2 corner list for a rectangle centred at pose(1:2), heading
% pose(3), full length dims.Length, full width dims.Width.
x = pose(1); y = pose(2); th = pose(3);
hl = dims.Length/2; hw = dims.Width/2;
localCorners = [ hl  hw; hl -hw; -hl -hw; -hl  hw];
R = [cos(th) -sin(th); sin(th) cos(th)];
corners = (R*localCorners')' + [x y];
end

function ax = edgeNormal(theta)
% Unit vector normal to an edge whose direction is `theta`.
ax = [-sin(theta) cos(theta)];
end

function [lo, hi] = projectOntoAxis(corners, ax)
p = corners * ax(:);
lo = min(p); hi = max(p);
end
