function veh = initVehicleState(x0, y0, theta0, v0, L)
% initVehicleState  Create initial ego vehicle state struct.
% Foundation for Stage 1 (bicycle-model vehicle simulation).
%
%   veh = initVehicleState(x0, y0, theta0, v0, L)
%
% Inputs:
%   x0, y0 - initial position [m]
%   theta0 - initial heading [rad]
%   v0     - initial speed [m/s]
%   L      - wheelbase [m] (default 2.7 if omitted)
%
% Output struct fields: x, y, theta, v, L

if nargin < 5 || isempty(L)
    L = 2.7;
end

veh.x = x0;
veh.y = y0;
veh.theta = theta0;
veh.v = v0;
veh.L = L;
end
