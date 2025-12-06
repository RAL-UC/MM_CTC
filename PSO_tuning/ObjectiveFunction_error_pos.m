function o = ObjectiveFunction_error_pos(x, ctrl_id)
%OBJECTIVEFUNCTION_ERROR_POS Cost function for discrete-time controller tuning.
%   o = ObjectiveFunction_error_pos(x, ctrl_id) evaluates the scalar cost J
%   for a given gain vector x = [Kp1..Kp5 Kv1..Kv5] and controller type:
%       ctrl_id = 'CTC'  -> uses 'Discrete_CTC.slx'
%       ctrl_id = 'PD'   -> uses 'Discrete_PD.slx'
%
%   The cost combines:
%       - RMS end-effector tracking error (Jtray)
%       - RMS control power (Jener)
%   into J = alpha * Jtray + beta * Jener, consistent with the formulation
%   described in the manuscript.
%
%   This function is intended to be used as the objective function inside
%   PSO2, via function handles such as:
%       problem.fobj = @(x) ObjectiveFunction_error_pos(x, 'CTC');
%       problem.fobj = @(x) ObjectiveFunction_error_pos(x, 'PD');

if nargin < 2
    error('ObjectiveFunction_error_pos requires two inputs: x and ctrl_id.');
end

% Select Simulink model based on controller ID
switch upper(ctrl_id)
    case 'CTC'
        modelName = 'Discrete_CTC.slx';
    case 'PD'
        modelName = 'Discrete_PD.slx';
    otherwise
        error('Unknown controller ID: %s. Use ''CTC'' or ''PD''.', ctrl_id);
end

fprintf('\nTesting gains for %s controller:\n', upper(ctrl_id));
fprintf('  Kp: [%.3f %.3f %.3f %.3f %.3f]\n', ...
        x(1), x(2), x(3), x(4), x(5));
fprintf('  Kv: [%.3f %.3f %.3f %.3f %.3f]\n', ...
        x(6), x(7), x(8), x(9), x(10));

% Reset sensor-noise seeds
rng('shuffle'); assignin('base','r1',abs(rand)*100);
rng('shuffle'); assignin('base','r2',abs(rand)*100);
rng('shuffle'); assignin('base','r3',abs(rand)*100);
rng('shuffle'); assignin('base','r4',abs(rand)*100);
rng('shuffle'); assignin('base','r5',abs(rand)*100);

% Reset actuator-noise seeds
rng('shuffle'); assignin('base','t1',abs(rand)*100);
rng('shuffle'); assignin('base','t2',abs(rand)*100);
rng('shuffle'); assignin('base','t3',abs(rand)*100);
rng('shuffle'); assignin('base','t4',abs(rand)*100);
rng('shuffle'); assignin('base','t5',abs(rand)*100);

% Assign gains to base workspace
assignin('base','Kp1',x(1));
assignin('base','Kp2',x(2));
assignin('base','Kp3',x(3));
assignin('base','Kp4',x(4));
assignin('base','Kp5',x(5));
assignin('base','Kv1',x(6));
assignin('base','Kv2',x(7));
assignin('base','Kv3',x(8));
assignin('base','Kv4',x(9));
assignin('base','Kv5',x(10));

% Run discrete-time Simulink model (controller selected above)
simout = sim(modelName, 'FastRestart', 'on');

N     = length(simout.xd(:,end));
xref  = simout.xd(:,2);
yref  = simout.xd(:,3);
zref  = simout.xd(:,4);
xmed  = simout.xa(:,2);
ymed  = simout.xa(:,3);
zmed  = simout.xa(:,4);

errorx = xref - xmed;
errory = yref - ymed;
errorz = zref - zmed;

enq1 = simout.energias(:,2);
enq2 = simout.energias(:,3);
enq3 = simout.energias(:,4);
enq4 = simout.energias(:,5);
enq5 = simout.energias(:,6);

sumtr = 0;
sumqs = 0;
for i = 1:N
    sumtr = sumtr + errorx(i)^2 + errory(i)^2 + errorz(i)^2;
    q1 = enq1(i); q2 = enq2(i); q3 = enq3(i); q4 = enq4(i); q5 = enq5(i);
    sumqs = sumqs + q1^2 + q2^2 + q3^2 + q4^2 + q5^2;
end

Jtray = sqrt((1/N) * sumtr);
Jener = sqrt((1/N) * sumqs);

alpha = 10;
beta  = 0.01;
J     = alpha * Jtray + beta * Jener;

fprintf('  Jtray = %.15f\n', Jtray);
fprintf('  Jener = %.15f\n', Jener);
fprintf('  Total cost J = %.15f\n\n', J);

o = J;
end
