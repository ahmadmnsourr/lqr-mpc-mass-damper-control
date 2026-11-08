%% Mass-damper control: LQR versus constrained MPC
% Main question: can MPC track a position step while respecting |u| <= 1?
% Based on the Digital Control Technologies and Architectures coursework.
% Requires Control System Toolbox and Optimization Toolbox.
%
% Run this file from the project folder. The two figures and a results
% table will be saved automatically. No Simulink model is required.

clear;
close all;
clc;

%% 1. Define and discretize the original mass-damper model
A = [0  1;
     0 -1];
B = [0; 1];
C = [1 0];
D = 0;

Ts = 0.05;                   % Controller sampling time, seconds
plant = ss(A, B, C, D);
plantDiscrete = c2d(plant, Ts, 'zoh');
Ad = plantDiscrete.A;
Bd = plantDiscrete.B;

assert(rank(ctrb(Ad, Bd)) == 2, 'The discrete plant is not controllable.');

%% 2. Design LQR with integral action
% The integral state accumulates position-tracking error.
% Original coursework weights: Q = diag([850 150 1]), R = 1.
Aaug = [1, -Ts*C;
        zeros(2,1), Ad];
Baug = [0; Bd];
Q = diag([850 150 1]);

K_lqr = dlqr(Aaug, Baug, Q, 1);

% Additional comparison, not an original saved coursework configuration:
% larger input penalty produces a gentler (usually slower) response.
K_lqr_tuned = dlqr(Aaug, Baug, Q, 100);

assert(all(abs(eig(Aaug - Baug*K_lqr)) < 1), 'LQR closed loop is unstable.');

%% 3. Prepare the constrained MPC
% Lab 6's original settings are kept, but the implementation below is new.
Hp = 25;                     % Prediction horizon
Hu = 25;                     % Control horizon
Qy = 120000;                 % Position tracking weight
Rdu = 120;                   % Penalty on changes in control input
uMin = -1;
uMax = 1;
yMax = 1.0099;               % Upper bound on predicted sampled position

% Use changes in force (du) as the optimization variables.
% z = [position; velocity; previous force].
A_mpc = [Ad, Bd;
         0, 0, 1];
B_mpc = [Bd; 1];
C_mpc = [C, 0];

% Future positions: Y = P*z + T*duSequence.
% P describes motion without new force changes.
% T describes the effect of each future force change.
P = zeros(Hp, 3);
T = zeros(Hp, Hu);

for i = 1:Hp
    P(i,:) = C_mpc * (A_mpc^i);
    for j = 1:min(i, Hu)
        T(i,j) = C_mpc * (A_mpc^(i-j)) * B_mpc;
    end
end

% Accumulate force changes to obtain the future absolute forces:
% U = previousForce + L*duSequence.
L = tril(ones(Hp, Hu));

% MATLAB quadprog solves (1/2)*du'*H*du + f'*du.
% Dividing the full MPC cost by Rdu keeps the same optimum and improves
% numerical scaling for these particular original weights.
weightRatio = Qy / Rdu;
H = 2*(weightRatio*(T'*T) + eye(Hu));
H = (H + H')/2;           % Remove small floating-point asymmetry
options = optimoptions('quadprog', 'Display', 'off', ...
                       'Algorithm', 'interior-point-convex');

%% 4. Simulate each controller on exactly the same plant
reference = 1;               % Unit-step position reference
simulationTime = 15;         % Extended for reliable settling-time checks
N = round(simulationTime/Ts);
time = (0:N)*Ts;

xLqr = zeros(2, N+1);
xTuned = zeros(2, N+1);
xMpc = zeros(2, N+1);
forceLqr = zeros(1, N);
forceTuned = zeros(1, N);
forceMpc = zeros(1, N);

errorIntegralLqr = 0;
errorIntegralTuned = 0;
previousForce = 0;           % Assumed input before t=0
maxQpResidual = 0;

for k = 1:N
    % Unconstrained original integral LQR.
    forceLqr(k) = -K_lqr*[errorIntegralLqr; xLqr(:,k)];

    % Same integral LQR with larger R (new illustrative tuning).
    forceTuned(k) = -K_lqr_tuned*[errorIntegralTuned; xTuned(:,k)];

    % MPC solves a constrained QP and applies only the first force move.
    [forceMpc(k), qpResidual] = solve_mpc( ...
        xMpc(:,k), previousForce, reference, ...
        P, T, L, H, weightRatio, uMin, uMax, yMax, options);
    maxQpResidual = max(maxQpResidual, qpResidual);

    % All three simulations use the SAME exact ZOH discrete plant.
    xLqr(:,k+1)  = Ad*xLqr(:,k)  + Bd*forceLqr(k);
    xTuned(:,k+1) = Ad*xTuned(:,k) + Bd*forceTuned(k);
    xMpc(:,k+1)  = Ad*xMpc(:,k)  + Bd*forceMpc(k);

    % Integrators use sampled position error (not a continuous integrator).
    errorIntegralLqr = errorIntegralLqr + Ts*(reference - C*xLqr(:,k));
    errorIntegralTuned = errorIntegralTuned + Ts*(reference - C*xTuned(:,k));
    previousForce = forceMpc(k);
end

assert(max(abs(forceMpc)) <= uMax + 1e-6, 'MPC exceeded the force limit.');
assert(maxQpResidual < 1e-5, 'QP numerical constraint check failed.');

%% 5. Calculate continuous output BETWEEN controller samples
% For this particular model, with a constant force over one sample:
% p(t+tau) = p(t) + (1-exp(-tau))*v(t) + (tau-1+exp(-tau))*u(t).
% Using this exact formula avoids missing overshoot between samples.
substeps = 50;               % 1 ms observation grid
smallTime = Ts/substeps;
tFine = (0:N*substeps)*smallTime;
tau = (0:substeps-1)*smallTime;
positionLqr = zeros(1, numel(tFine));
positionTuned = zeros(1, numel(tFine));
positionMpc = zeros(1, numel(tFine));

for k = 1:N
    idx = (k-1)*substeps + (1:substeps);
    decay = exp(-tau);
    positionLqr(idx) = xLqr(1,k) + (1-decay)*xLqr(2,k) ...
        + (tau-1+decay)*forceLqr(k);
    positionTuned(idx) = xTuned(1,k) + (1-decay)*xTuned(2,k) ...
        + (tau-1+decay)*forceTuned(k);
    positionMpc(idx) = xMpc(1,k) + (1-decay)*xMpc(2,k) ...
        + (tau-1+decay)*forceMpc(k);
end

positionLqr(end) = xLqr(1,end);
positionTuned(end) = xTuned(1,end);
positionMpc(end) = xMpc(1,end);

%% 6. Calculate performance and save ONE comparison table
names = {'Integral LQR (original)'; ...
         'Integral LQR (R=100, new tuning)'; ...
         'Constrained MPC (original settings)'};
outputs = {positionLqr, positionTuned, positionMpc};
forces = {forceLqr, forceTuned, forceMpc};
settlingTime = zeros(3,1);
peakForce = zeros(3,1);
controlEnergy = zeros(3,1);
overshootPercent = zeros(3,1);

for i = 1:3
    deviation = abs(outputs{i} - reference);
    lastOutside = find(deviation > 0.01*reference, 1, 'last');
    if isempty(lastOutside)
        settlingTime(i) = 0;
    elseif lastOutside == numel(tFine)
        settlingTime(i) = NaN;
    else
        settlingTime(i) = tFine(lastOutside+1);
    end
    peakForce(i) = max(abs(forces{i}));
    controlEnergy(i) = Ts*sum(forces{i}.^2);
    overshootPercent(i) = 100*max(0, max(outputs{i})-reference)/reference;
end

results = table(names, settlingTime, peakForce, controlEnergy, ...
                overshootPercent, 'VariableNames', ...
                {'Controller','SettlingTime_s','PeakForce', ...
                 'ControlEnergy','Overshoot_percent'});
disp(results);
fprintf('MPC maximum sampled force: %.6f\n', peakForce(3));
fprintf('MPC QP maximum inequality residual: %.3g\n', maxQpResidual);
fprintf('MPC intersample output-bound exceedance: %.6g\n', ...
        max(0, max(positionMpc)-yMax));

if ~exist('results','dir'), mkdir('results'); end
writetable(results, fullfile('results','matlab_comparison.csv'));

%% 7. Save ONLY the two figures needed to answer the research question
if ~exist('figures','dir'), mkdir('figures'); end

figure('Color','w');
h1 = plot(tFine, positionLqr, 'LineWidth',1.6); hold on;
h2 = plot(tFine, positionTuned, 'LineWidth',1.6);
h3 = plot(tFine, positionMpc, 'LineWidth',1.6);
yline(reference, 'k--', 'Reference');
yline(reference+0.01, ':', '1% tolerance');
yline(reference-0.01, ':');
xlim([0 8]); grid on;
xlabel('Time (s)'); ylabel('Position');
title('Unit-step position tracking');
legend([h1 h2 h3], names, 'Location','southeast');
saveas(gcf, fullfile('figures','tracking_matlab.png'));

figure('Color','w');
f1 = stairs(time(1:end-1), forceLqr, 'LineWidth',1.4); hold on;
f2 = stairs(time(1:end-1), forceTuned, 'LineWidth',1.4);
f3 = stairs(time(1:end-1), forceMpc, 'LineWidth',1.4);
yline(uMax, 'k--', 'Input limits');
yline(uMin, 'k--');
xlim([0 8]); grid on;
xlabel('Time (s)'); ylabel('Applied force');
title('Control effort and force constraints');
legend([f1 f2 f3], names, 'Location','northeast');
saveas(gcf, fullfile('figures','force_matlab.png'));

fprintf('\nSaved: results/matlab_comparison.csv and two MATLAB figures.\n');
