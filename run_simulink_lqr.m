%% Optional Simulink demonstration: LQR with integral action
% This is the original coursework Simulink architecture for step tracking.
% It is separate from run_project.m, which compares LQR and MPC in MATLAB.
%
% Requirements: MATLAB, Simulink and Control System Toolbox.
% This model has not yet been executed in MATLAB in this repository.

clear;
clc;

%% 1. Define the mass-damper system
A = [0 1; 0 -1];
B = [0; 1];
C = [1 0];
D = 0;

Ts = 0.05;
x0 = [0; 0];              % Plant initial state
x0i = 0;                  % Integrator initial state
t_sim = 15;               % Simulation time in seconds

%% 2. Calculate the discrete LQR gain with integral action
plant = ss(A, B, C, D);
plantDiscrete = c2d(plant, Ts, 'zoh');
Ad = plantDiscrete.A;
Bd = plantDiscrete.B;

Aaug = [1 -Ts*C;
        zeros(2,1) Ad];
Baug = [0; Bd];
Q = diag([850 150 1]);
R = 1;

K = dlqr(Aaug, Baug, Q, R);
K_q = K(1);               % Integral gain
K_x = K(2:3);             % Position and velocity gains

% The Simulink plant block outputs both states, not only position.
sys_x = ss(A, B, eye(2), zeros(2,1));

%% 3. Run the original continuous-plant / sampled-control architecture
% The model contains: Step, sampled error integrator, feedback gains,
% zero-order hold, continuous-time plant, and workspace outputs.
modelFile = fullfile(fileparts(mfilename('fullpath')), ...
    'simulink', 'integral_lqr_tracking.slx');
load_system(modelFile);

% The model reads variables such as K_x, K_q, Ts and sys_x from the
% MATLAB base workspace. Run this file as a script, not a function.
out = sim('integral_lqr_tracking');

%% 4. Read the saved Simulink results
position = out.y.Data;
inputForce = out.u.Data;

fprintf('Simulink integral LQR result (original R = 1):\n');
fprintf('Final position: %.6f\n', position(end));
fprintf('Maximum commanded force: %.6f\n', max(abs(inputForce(:))));
fprintf('This LQR is unconstrained; its force can exceed +/-1.\n');

% To inspect the actual block diagram, run:
% open_system('integral_lqr_tracking')
