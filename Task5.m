%% Task 5: Set-Point Tracking with Integral Action
% Matriculation: 8246
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

% 2. Design Integral LQR
% Augmented State Vector: z = [x; xi] (6x1 vector)
% dot_xi = r - y = r - Cx
% Augmented system matrix:
% [dot_x ]   [ A    0 ] [ x ]   [ B ]       [ 0 ]
% [dot_xi] = [ -C   0 ] [ xi] + [ 0 ] u  +  [ I ] r

n = 4; % number of states
p = 2; % number of outputs/integrators

A_aug = [A, zeros(n, p); -C, zeros(p, p)];
B_aug = [B; zeros(p, 2)];

% Tuning Weights for LQR
% We need to penalize x (keep it bounded), xi (eliminate error), and u
Q_x = 1 * eye(n);       % Weight on states (small penalty)
Q_xi = 1000 * eye(p);   % Weight on integral error (HIGH penalty -> drives error to 0)
Q_aug = blkdiag(Q_x, Q_xi);

R_aug = 10 * eye(2);    % Weight on control inputs

% Calculate Optimal Gain
[K_aug, ~, ~] = lqr(A_aug, B_aug, Q_aug, R_aug);

% Separate Gain into K_x (State Feedback) and K_i (Integral Gain)
K_x = K_aug(:, 1:n);
K_i = K_aug(:, n+1:end);

fprintf('State Feedback Gain K_x:\n'); disp(K_x);
fprintf('Integral Gain K_i:\n'); disp(K_i);

% 3. Simulation Setup
% Target Set-Point
y_sp = [0.4; 0.8];
% Disturbance w = [0.3, 0.2] starting at t = 10s
w_dist = [0.3; 0.2];
t_dist = 10;

% Simulation Loop (Manual Euler/RK4 or lsim with custom input)
% Since disturbance is a step at t=10, we can use lsim construction.
% The plant equation with input disturbance: dot_x = Ax + B(u + w)
% u = -K_x*x - K_i*xi
% Closed loop:
% dot_x = (A - B*K_x)x - B*K_i*xi + B*w
% dot_xi = r - Cx

% construct the closed-loop augmented system with inputs [r; w]
% State z = [x; xi]
% Inputs: [r (2x1); w (2x1)]
A_cl = [A - B*K_x, -B*K_i; -C, zeros(p,p)];
B_cl = [zeros(n,2), B; eye(p), zeros(p,2)]; 
C_cl = [C, zeros(p,p)]; % We want to look at output y

sys_final = ss(A_cl, B_cl, C_cl, zeros(2,4));

% Time vector
t = 0:0.01:30; % 30s simulation

% Input signals
u_sim = zeros(length(t), 4); 
% Columns 1-2: Reference r (Constant)
u_sim(:, 1) = y_sp(1);
u_sim(:, 2) = y_sp(2);
% Columns 3-4: Disturbance w (Step at t=10)
dist_indices = t >= t_dist;
u_sim(dist_indices, 3) = w_dist(1);
u_sim(dist_indices, 4) = w_dist(2);

% Simulate
[y_out, t, z_out] = lsim(sys_final, u_sim, t);

% Extract Control Signals for plotting
% u = -K_x*x - K_i*xi (Note: disturbance is added to plant, not controller)
x_traj = z_out(:, 1:n);
xi_traj = z_out(:, n+1:end);
u_ctrl = zeros(length(t), 2);
for i = 1:length(t)
    u_ctrl(i,:) = (-K_x * x_traj(i,:)' - K_i * xi_traj(i,:)')';
end

% 4. Plotting
% Figure 8.1: Output Tracking
figure('Name', 'Fig 8.1: Set-Point Tracking & Disturbance Rejection');
subplot(2,1,1);
plot(t, y_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t, u_sim(:,1), 'k--', 'LineWidth', 1); % Reference
xline(t_dist, 'g:', 'Disturbance');
title('Output y_1 (AFR)'); ylabel('Amplitude'); grid on; legend('Response', 'Set-Point');

subplot(2,1,2);
plot(t, y_out(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t, u_sim(:,2), 'k--', 'LineWidth', 1); % Reference
xline(t_dist, 'g:', 'Disturbance');
title('Output y_2 (EGR %)'); ylabel('Amplitude'); grid on; legend('Response', 'Set-Point');

% Figure 8.2: Control Inputs
figure('Name', 'Fig 8.2: Control Inputs (Task 5)');
subplot(2,1,1); plot(t, u_ctrl(:,1), 'r'); title('Control u_1 (VGT)'); grid on; xline(t_dist, 'g:');
subplot(2,1,2); plot(t, u_ctrl(:,2), 'r'); title('Control u_2 (EGR)'); grid on; xline(t_dist, 'g:');