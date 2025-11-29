%% Task 6: State Regulation and Optimization
% Matriculation:8246
clc; clear; close all;

% 1. Define System
A = [-7.6487, -0.0399, -4.5500, 3.5846;
     -4.5740, 3.0568, -4.3662, -1.3183;
     3.7698, 15.3212, -15.8770, 4.4936;
     -9.0645, 8.3742, -4.4331, -9.9233];
B = [0.1993, 0.0319;
     0.0122, -0.0200;
     4.4939, 2.0552;
     -1.4269, -0.2730];
C = [-3.2988, -1.8646, 0.0370, -0.0109;
     0.2602, -2.1506, -0.0104, 0.0163];
D = zeros(2,2);

% Student Parameters (for Weight Matrix)
a = 8; b = 2; c = 4; d = 6;

% 2. Problem Setup
% Target State Set-Point (Unreachable)
x_sp = [0; 0.5; -0.4; 0.3];

% Feasibility Check
% At steady state: Ax + Bu = 0  =>  x = -inv(A)*B*u
% We want x = x_sp. 
% Check if -inv(A)*B*u = x_sp is solvable.
M = -inv(A) * B; % Mapping from u_ss to x_ss

% 3. Optimization (Weighted Least Squares)
% Objective: Minimize J = 0.5 * (x_s - x_sp)' * W * (x_s - x_sp)
% Subject to: x_s = M * u_s
% This becomes: Minimize || M*u_s - x_sp ||_W

% Define Weight Matrix W = diag(a+1, b+1, c+1, d+1)
W_diag = [a+1, b+1, c+1, d+1];
W = diag(W_diag);

fprintf('Weight Matrix W:\n');
disp(W);

% Solution for u_opt using Weighted Least Squares:
% u_opt = (M' * W * M)^-1 * M' * W * x_sp
u_opt = inv(M' * W * M) * M' * W * x_sp;

% Calculate the achievable steady state x_target
x_target = M * u_opt;

fprintf('Desired State x_sp:\n'); disp(x_sp');
fprintf('Achievable State x_target (Optimized):\n'); disp(x_target');
fprintf('Required Steady Control u_opt:\n'); disp(u_opt');

% 4. Controller Implementation
% Control Law: u(t) = -K * (x(t) - x_target) + u_opt
% We use LQR Gain from Task 2 (Design B - Efficient) for stability
Q_lqr = 1000 * eye(4); 
R_lqr = 100 * eye(2);
[K, ~, ~] = lqr(A, B, Q_lqr, R_lqr);

% 5. Simulation
sys_cl = ss(A - B*K, B, C, D);

% Simulation Parameters
t = 0:0.01:10;
x0 = [0; 0; 0; 0]; % Start from zero (or any condition)

% We need to simulate the offset dynamics.
% dot_x = Ax + B( -K(x - x_target) + u_opt )
% dot_x = (A - BK)x + (BK*x_target + B*u_opt)
% Define an effective reference input "r_eff" for the closed loop
r_eff = B * K * x_target + B * u_opt;

% Create a state-space for simulation: dot_x = A_cl*x + I*r_eff
sys_sim = ss(A - B*K, eye(4), C, zeros(2,4));

% Input is constant r_eff over time
u_sim = repmat(r_eff', length(t), 1);

[y_sim, t, x_sim] = lsim(sys_sim, u_sim, t, x0);

% 6. Plotting
figure('Name', 'Fig 9.1: State Regulation Optimization');

state_names = {'x_1', 'x_2', 'x_3', 'x_4'};
for i = 1:4
    subplot(2,2,i);
    plot(t, x_sim(:, i), 'b', 'LineWidth', 1.5); hold on;
    yline(x_sp(i), 'r:', 'LineWidth', 1.5);       % The Impossible Target
    yline(x_target(i), 'g--', 'LineWidth', 1.5);  % The Optimized Target
    title(['State ' state_names{i}]);
    xlabel('Time (s)'); grid on;
    if i == 1
        legend('Actual Response', 'Desired Set-Point (x_{sp})', 'Optimized Target (x_{target})');
    end
end

% Output numerical error for report
final_error = x_target - x_sp;
weighted_cost = 0.5 * final_error' * W * final_error;
fprintf('Final Weighted Cost J: %.4f\n', weighted_cost);