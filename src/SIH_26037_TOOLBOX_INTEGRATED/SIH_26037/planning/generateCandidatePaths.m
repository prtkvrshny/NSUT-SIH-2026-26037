function candidates = generateCandidatePaths(veh, centerline, params)
% generateCandidatePaths  Build a fan of lateral-offset candidate paths.
% Supports Stage 2 (candidate path generation) and Phase E item 19
% (multi-obstacle path evaluation operates on this candidate set).
%
% *** No lane-following assumption (Stage 4 requirement) ***: candidate
% lateral offsets are sampled continuously across the drivable corridor
% width, not snapped to discrete lanes.
%
%   candidates = generateCandidatePaths(veh, centerline, params)
%
% params: .corridorHalfWidth (default 4 m), .numCandidates (default 9),
%   .horizonDist (default 30 m), .numSamples (default 20),
%   .transitionDist (default = horizonDist) -- see below.
%
% Output: 1xN struct array, each with fields:
%   .offset - target lateral offset (d) at the end of the horizon
%   .xy     - Mx2 array of [x y] points along this candidate
%   .s      - Mx1 arc-length values matching xy
%
% *** transitionDist matters a lot for close obstacles ***: the lateral
% offset is reached via a smoothstep over `transitionDist` metres, not
% necessarily the full `horizonDist` used for sampling. If a threat is
% only, say, 9 m ahead but transitionDist were left at the default 30 m,
% EVERY candidate would still be within ~20% of its way through the
% lateral maneuver by the time it reaches that obstacle -- none of them
% would actually have moved out of the way yet, making all candidates
% look infeasible even though a real swerve was available. Callers
% (planning/adaptivePlanner.m) are expected to shrink transitionDist
% toward the nearest known threat's distance (with a safety margin and a
% physically-sane floor) precisely to avoid this. This was found via
% Octave smoke-testing during development (see NOTES.md) -- the default
% here intentionally still equals horizonDist so a caller that does NOT
% set transitionDist gets the ORIGINAL (buggy-for-close-obstacles)
% behavior, rather than a silently different one it never asked for.

if nargin < 3, params = struct(); end
halfW          = getOrDefault(params, 'corridorHalfWidth', 4.0);
numC           = getOrDefault(params, 'numCandidates', 9);
horizonDist    = getOrDefault(params, 'horizonDist', 30);
numSamples     = getOrDefault(params, 'numSamples', 20);
transitionDist = getOrDefault(params, 'transitionDist', horizonDist);
transitionDist = max(transitionDist, 1.0); % avoid divide-by-near-zero

[s0, d0] = frenetUtils('cart2frenet', centerline, veh.x, veh.y);

offsets = linspace(-halfW, halfW, numC);
candidates = struct('offset', {}, 'xy', {}, 's', {});

sSamples = linspace(s0, s0 + horizonDist, numSamples)';

for k = 1:numC
    dTarget = offsets(k);
    frac = (sSamples - s0) / transitionDist;
    frac = max(0, min(1, frac));
    smoothFrac = 3*frac.^2 - 2*frac.^3; % smoothstep: avoids a lateral jerk step
    dProfile = d0 + (dTarget - d0) .* smoothFrac;

    xy = zeros(numSamples, 2);
    for m = 1:numSamples
        [xx, yy, ~] = frenetUtils('frenet2cart', centerline, sSamples(m), dProfile(m));
        xy(m,:) = [xx yy];
    end

    c.offset = dTarget;
    c.xy = xy;
    c.s = sSamples;
    candidates(end+1) = c;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
