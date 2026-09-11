function [a, delta] = emergencyBraking(veh, maxDecel)
% emergencyBraking  Maximum-deceleration, wheels-straight braking command.
% Implements Phase E item 20.
%
%   [a, delta] = emergencyBraking(veh, maxDecel)

if nargin < 2 || isempty(maxDecel), maxDecel = 7.0; end
a = -abs(maxDecel);
delta = 0;
end
