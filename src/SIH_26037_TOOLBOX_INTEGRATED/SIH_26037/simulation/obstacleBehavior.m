function obs = obstacleBehavior(obs, dt, t, roadInfo)
% obstacleBehavior  Type/behavior-specific motion intent for one obstacle.
% Implements Phase A item 3 (object types) and Phase D item 15
% (pedestrian/bike/auto/pushcart/cattle behavior patterns).
%
%   obs = obstacleBehavior(obs, dt, t, roadInfo)
%
% Updates obs.theta and obs.v (heading & speed INTENT for this step);
% simulation/obstacleUpdate.m integrates position from these.
%
% roadInfo (.centerline, .halfWidth) is passed through for future
% behaviors that might want to react to road extent -- current behaviors
% do not clamp obstacles to any boundary, so obstacles can in principle
% wander outside the nominal corridor over a long run (see NOTES.md).

switch obs.behavior
    case 'static'
        obs.v = 0;

    case 'straightLine'
        % constant velocity, no change

    case 'crossing'
        % Pedestrian/vehicle crossing roughly perpendicular to the
        % centerline, with small heading jitter for realism.
        obs.theta = obs.theta + 0.05*randn()*dt;

    case 'weaving'
        % Bike-like lateral weave: sinusoidal heading oscillation about
        % a nominal heading captured on first use.
        if ~isfield(obs.params,'baseTheta'), obs.params.baseTheta = obs.theta; end
        obs.theta = obs.params.baseTheta + deg2rad(15)*sin(0.8*t);

    case 'randomWalk'
        % Pedestrian/cattle unstructured wandering: random heading
        % perturbation + occasional pause/resume.
        obs.theta = obs.theta + 0.4*randn()*dt;
        if rand() < 0.2*dt
            obs.v = 0;
        elseif obs.v == 0 && rand() < 0.3
            obs.v = obs.params.nominalSpeed;
        end

    case 'erratic'
        % Cattle: mostly slow/stationary, can suddenly start moving in a
        % new random direction -- the behavior most likely to defeat a
        % naive constant-velocity-only planner.
        if rand() < 0.1*dt
            obs.theta = obs.theta + (rand()-0.5)*pi;
            obs.v = obs.params.nominalSpeed * (0.3 + 0.7*rand());
        end

    otherwise
        error('obstacleBehavior:unknown', 'Unknown behavior "%s"', obs.behavior);
end

obs.vx = obs.v*cos(obs.theta);
obs.vy = obs.v*sin(obs.theta);
end
