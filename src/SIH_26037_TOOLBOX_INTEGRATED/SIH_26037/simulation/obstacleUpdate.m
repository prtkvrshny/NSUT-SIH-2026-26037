function obstacles = obstacleUpdate(obstacles, dt, t, roadInfo)
% obstacleUpdate  Advance every obstacle by one simulation step.
% Implements Phase A item 4 (obstacle motion/update) and activates any
% obstacle whose spawnTime has arrived (Phase B item 8 support).
%
%   obstacles = obstacleUpdate(obstacles, dt, t, roadInfo)

for k = 1:numel(obstacles)
    if ~obstacles(k).active
        if t >= obstacles(k).spawnTime
            obstacles(k).active = true;
        else
            continue;
        end
    end
    obstacles(k) = obstacleBehavior(obstacles(k), dt, t, roadInfo);
    obstacles(k).x = obstacles(k).x + obstacles(k).vx*dt;
    obstacles(k).y = obstacles(k).y + obstacles(k).vy*dt;
end
end
