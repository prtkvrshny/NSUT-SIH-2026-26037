function dims = obstacleFootprints(type)
% obstacleFootprints  Per-type physical footprint (Length, Width) for
% collision geometry, backed by Automated Driving Toolbox's
% vehicleDimensions object where available.
%
% *** TOOLBOX ADD-ON (see TOOLBOX_INTEGRATION.md) ***
% This directly targets a weak point flagged in the original NOTES.md:
% "Flat collision radius (1.5 m for every type): no per-type footprint.
% A cattle-sized obstacle and a pedestrian get identical clearance
% requirements. Deliberately not fixed, to avoid adding untested
% geometry code." Real vehicleDimensions objects (or, on a system
% without Automated Driving Toolbox / a valid license, a plain struct
% with the SAME field names) now give an honest per-type size instead.
%
%   dims = obstacleFootprints(type)
%
% type: 'pedestrian'|'bike'|'auto'|'pushcart'|'cattle'|'unknown'|'ego'
%
% Output: an object/struct with fields .Length, .Width (metres) --
% either a real vehicleDimensions object (has other properties too,
% e.g. .Height/.FrontOverhang/.RearOverhang, unused here) or, in the
% fallback path, a plain struct with just .Length and .Width. Callers
% (planning/obbCollisionCheck.m) only ever read .Length/.Width, so
% both forms are interchangeable for this project's purposes.
%
% CONFIDENCE NOTE: vehicleDimensions is a real, stable Automated Driving
% Toolbox object; I'm confident in the constructor call shape used here
% (I've seen it consistently across many toolbox examples). What I
% CANNOT verify from this sandbox is Automated Driving Toolbox licensing
% on your machine specifically -- the try/catch fallback exists
% precisely so a licensing surprise degrades gracefully (same numbers,
% plain struct) instead of erroring out your whole planner. The
% fallback struct path is the one actually exercised by Octave testing
% in this sandbox (Octave has no vehicleDimensions at all) -- see
% TOOLBOX_INTEGRATION.md for exactly what was and wasn't run.
%
% Footprint sizes below are reasonable engineering estimates for
% Indian-road context (auto-rickshaw, loaded pushcart, cattle, etc.),
% NOT measured data -- treat them as a starting point to tune, not
% ground truth.

sizes = struct( ...
    'pedestrian', [0.6 0.6], ...
    'bike',       [1.8 0.6], ...
    'auto',       [3.2 1.4], ...
    'pushcart',   [1.5 1.0], ...
    'cattle',     [2.0 0.9], ...
    'unknown',    [1.5 1.0], ...
    'ego',        [4.0 1.8]);

if isfield(sizes, type)
    LW = sizes.(type);
else
    LW = sizes.unknown;
end

try
    dims = vehicleDimensions(LW(1), LW(2));
catch
    % No Automated Driving Toolbox (e.g. this sandbox's Octave, or a
    % MATLAB seat without the license) -- plain-struct fallback with
    % the same field names used by planning/obbCollisionCheck.m.
    dims.Length = LW(1);
    dims.Width  = LW(2);
end
end
