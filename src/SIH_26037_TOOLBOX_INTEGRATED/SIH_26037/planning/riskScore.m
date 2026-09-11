function [risk, details] = riskScore(veh, track, ttc, params)
% riskScore  Scalar risk score in [0,1] for one tracked obstacle.
% Implements Phase D items 15-16 (type-based danger weighting + risk score).
%
%   [risk, details] = riskScore(veh, track, ttc, params)
%
% Combines: inverse TTC, inverse distance, a per-type danger/
% unpredictability weight (Phase D item 15: pedestrian/bike/auto/
% pushcart/cattle behavior), and a confidence penalty for unconfirmed
% tracks. This last term interacts directly with bug-fix #2: a track we
% cannot yet trust the velocity of is scored as MORE dangerous, never
% less, precisely so a freshly-appeared obstacle cannot be waved through
% as "safe" before its motion is actually known.
%
% Type danger weights are a simplified stand-in for "how bad an outcome
% / how unpredictable": pedestrians and cattle are weighted highest
% (unpredictable, vulnerable, and can force a full stop); autos/bikes
% next (faster but more committed to their heading); pushcarts lowest
% (slow, near-static). See NOTES.md for the honest limits of this
% single-number model.

danger = struct('pedestrian',1.0,'bike',0.7,'auto',0.6,'pushcart',0.4,'cattle',0.9,'unknown',0.8);
if nargin < 4, params = struct(); end
ttcCap = getOrDefault(params,'ttcCap',8);

dist = hypot(track.x-veh.x, track.y-veh.y);
distTerm = 1 / (1 + max(dist,0)/10);

if isinf(ttc)
    ttcTerm = 0;
else
    ttcTerm = max(0, 1 - ttc/ttcCap);
end

if isfield(danger, track.type)
    typeW = danger.(track.type);
else
    typeW = danger.unknown;
end

confidencePenalty = 0;
if ~track.confirmed
    confidencePenalty = 0.25;
end

raw = 0.5*ttcTerm + 0.3*distTerm + 0.2*typeW + confidencePenalty;
risk = max(0, min(1, raw));

details = struct('dist', dist, 'ttc', ttc, 'typeWeight', typeW, ...
                  'confirmed', track.confirmed, 'ttcTerm', ttcTerm, ...
                  'distTerm', distTerm);
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
