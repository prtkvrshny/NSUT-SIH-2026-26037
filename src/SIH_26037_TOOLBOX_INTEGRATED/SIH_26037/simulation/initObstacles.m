function obstacles = initObstacles(obsConfigs)
% initObstacles  Build the ground-truth obstacle array from a scenario
% config's obstacle list.
% Implements Phase A items 1-3 (multiple obstacles, static+dynamic, types).
%
%   obstacles = initObstacles(obsConfigs)
%
% obsConfigs: struct array (from simulation/envConfigs.m or a stage
% script), each with:
%   .type ('pedestrian'|'bike'|'auto'|'pushcart'|'cattle')
%   .x0,.y0,.theta0,.v0 - initial pose/speed
%   .behavior           - behavior tag, see obstacleBehavior.m
%   .spawnTime          - time [s] the obstacle becomes active (0 =
%                          present from the start; >0 = Phase B item 8
%                          "surprise" obstacle)
%   .params             - behavior-specific scratch struct (optional)
%
% Output: obstacles struct array with fields id,type,x,y,theta,v,vx,vy,
%   behavior,spawnTime,active,params

obstacles = struct('id',{},'type',{},'x',{},'y',{},'theta',{},'v',{}, ...
                    'vx',{},'vy',{},'behavior',{},'spawnTime',{}, ...
                    'active',{},'params',{});

for k = 1:numel(obsConfigs)
    c = obsConfigs(k);
    o.id = k;
    o.type = c.type;
    o.x = c.x0; o.y = c.y0; o.theta = c.theta0; o.v = c.v0;
    o.vx = c.v0*cos(c.theta0); o.vy = c.v0*sin(c.theta0);
    o.behavior = c.behavior;
    o.spawnTime = getOrDefault(c, 'spawnTime', 0);
    o.active = (o.spawnTime <= 0);
    o.params = getOrDefault(c, 'params', struct());
    obstacles(end+1) = o;
end
end

function v = getOrDefault(s,f,default)
if isfield(s,f) && ~isempty(s.(f)), v = s.(f); else, v = default; end
end
