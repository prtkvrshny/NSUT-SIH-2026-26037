function plotResults(log, obstacles, cfg, envName)
% plotResults  Basic trajectory + telemetry plot, saved to results/.
% Visualization support for stage demos and Phase F/G runs; not itself
% a numbered checklist item.

figure('Name', sprintf('Scenario: %s', envName), 'Position',[100 100 900 700]);

subplot(2,1,1);
plot(log.vehX, log.vehY, 'b-', 'LineWidth', 1.5); hold on;
for k = 1:numel(obstacles)
    o = obstacles(k);
    plot(o.x, o.y, 'rx', 'MarkerSize', 8);
    text(o.x, o.y, [' ' o.type]);
end
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('%s -- ego trajectory (final obstacle positions marked)', envName));
axis equal; grid on;

subplot(2,1,2);
plot(log.t, log.vehV, 'b-'); hold on;
plot(log.t, log.risk*max(log.vehV+1e-6), 'r-');
xlabel('t [s]'); ylabel('speed [m/s]  /  risk (rescaled)');
legend('speed','risk (rescaled to plot on same axis)');
title('speed & risk over time');
grid on;

if ~exist('results','dir'), mkdir('results'); end
try
    saveas(gcf, fullfile('results', sprintf('%s_trajectory.png', envName)));
catch err
    fprintf('plotResults: could not save figure (%s) -- continuing without it.\n', err.message);
end
end
