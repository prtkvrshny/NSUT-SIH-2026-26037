function vehNext = bicycleModel(veh, a, delta, dt)
% bicycleModel  Kinematic bicycle model, one forward-Euler step.
% Implements Stage 1: base vehicle simulation (plain MATLAB, no toolbox).
%
%   vehNext = bicycleModel(veh, a, delta, dt)
%
% Inputs:
%   veh   - vehicle state struct (x,y,theta,v,L)
%   a     - longitudinal acceleration command [m/s^2]
%   delta - front wheel steering angle command [rad]
%   dt    - integration timestep [s]
%
% Output:
%   vehNext - updated vehicle state struct
%
%   xdot     = v*cos(theta)
%   ydot     = v*sin(theta)
%   thetadot = (v/L)*tan(delta)
%   vdot     = a

maxDelta = deg2rad(35);
delta = max(min(delta, maxDelta), -maxDelta);

vehNext = veh;
vehNext.x     = veh.x + veh.v*cos(veh.theta)*dt;
vehNext.y     = veh.y + veh.v*sin(veh.theta)*dt;
vehNext.theta = veh.theta + (veh.v/veh.L)*tan(delta)*dt;
vehNext.v     = max(veh.v + a*dt, 0);
end
