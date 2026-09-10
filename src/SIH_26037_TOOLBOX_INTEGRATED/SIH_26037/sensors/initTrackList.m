function tracks = initTrackList()
% initTrackList  Empty track struct array with the exact field set
% expected by sensors/trackObstacles.m. Call once before the first
% tracking update in any scenario.
tracks = struct('id',{},'x',{},'y',{},'vx',{},'vy',{},'type',{}, ...
                 'confirmed',{},'age',{},'missedFrames',{});
end
