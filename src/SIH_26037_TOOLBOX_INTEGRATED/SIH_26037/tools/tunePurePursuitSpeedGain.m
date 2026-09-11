function results = tunePurePursuitSpeedGain(targetSettlingTime)
% tunePurePursuitSpeedGain  Control System Toolbox OFFLINE design aid
% for the proportional speed-controller gain used in
% vehicle/purePursuitControl.m (currently hardcoded there as kP=1.0).
%
% *** TOOLBOX ADD-ON -- see TOOLBOX_INTEGRATION.md ***
% THIS SCRIPT DOES NOT RUN INSIDE THE SIMULATION AND DOES NOT CHANGE
% ANY RUNTIME BEHAVIOR. vehicle/purePursuitControl.m's kP=1.0 was
% validated by extensive Octave testing across all 5 environments (see
% NOTES.md) and is intentionally left untouched -- re-tuning it is a
% decision for you to make deliberately, informed by this script's
% output, not something this delivery does automatically. This script
% is here to demonstrate/justify a value formally with Control System
% Toolbox rather than by hand-tuning alone, and to give you a
% documented way to re-derive kP if you ever change the target
% dynamics (settling time / overshoot spec).
%
%   results = tunePurePursuitSpeedGain(targetSettlingTime)
%
% targetSettlingTime: desired 2% settling time [s] for the closed-loop
%   speed response (default 2.0 s, i.e. "reach within 2% of the
%   commanded target speed within about 2 seconds").
%
% Model: the longitudinal speed loop is v_dot = a, a = kP*(vTarget-v)
% (see vehicle/purePursuitControl.m and vehicle/bicycleModel.m) --
% exactly a plant G(s) = 1/s (acceleration command -> speed) in unity
% negative feedback with a pure proportional controller C(s) = kP. This
% is a textbook first-order closed loop with pole at s=-kP, i.e.
% closed-loop time constant tau=1/kP and 2%-settling time ~= 4/kP.
%
% Output `results` struct: .kP (recommended gain), .settlingTime
% (achieved, from step()), .closedLoopPole, .plant, .controller,
% .closedLoopSys (Control System Toolbox LTI objects, for your own
% further analysis/plotting).
%
% CONFIDENCE: the underlying control theory (first-order plant, P
% controller, pole/settling-time relationship) is exact and not
% toolbox-dependent -- the closed-form kP = 4/targetSettlingTime is
% the actual answer regardless of whether Control System Toolbox
% agrees. What IS toolbox-specific and UNTESTED here is the exact
% tf()/feedback()/step()/pidtune() call shapes below: this sandbox has
% no Control System Toolbox (Octave's optional `control` package could
% not be installed either -- no internet access to Octave Forge from
% here), so none of the toolbox calls in this file have ever been
% executed. The closed-form numeric answer these calls are meant to
% confirm has been checked by hand.

if nargin < 1 || isempty(targetSettlingTime)
    targetSettlingTime = 2.0;
end

% Closed-form answer (exact, not toolbox-dependent): 2% settling time
% of a first-order system with time constant tau is ~= 4*tau.
kP_closedForm = 4 / targetSettlingTime;

results.kP = kP_closedForm;
results.closedLoopPole = -kP_closedForm;
results.settlingTime = targetSettlingTime; % by construction of the closed-form formula
results.plant = [];
results.controller = [];
results.closedLoopSys = [];

try
    plant = tf(1, [1 0]);          % G(s) = 1/s
    controller = tf(kP_closedForm, 1); % C(s) = kP (pure proportional)
    closedLoopSys = feedback(controller*plant, 1);

    stepInfo = step(closedLoopSys); %#ok<NASGU> % sanity-run; not otherwise used
    S = stepinfo(closedLoopSys);

    results.plant = plant;
    results.controller = controller;
    results.closedLoopSys = closedLoopSys;
    results.settlingTime = S.SettlingTime; % toolbox-measured, should match the closed-form value above

    fprintf('Recommended kP = %.3f for a %.2f s settling time.\n', kP_closedForm, targetSettlingTime);
    fprintf('Control System Toolbox stepinfo() settling time: %.3f s (closed-form estimate: %.3f s)\n', ...
        S.SettlingTime, targetSettlingTime);
catch ME
    warning('tunePurePursuitSpeedGain:toolboxUnavailable', ...
        ['Control System Toolbox call failed (%s: %s) -- returning the ' ...
         'closed-form answer only (kP = 4/targetSettlingTime), which is ' ...
         'exact for this first-order plant regardless.'], ME.identifier, ME.message);
    fprintf('Recommended kP = %.3f for a %.2f s settling time (closed-form; toolbox unavailable to cross-check).\n', ...
        kP_closedForm, targetSettlingTime);
end
end
