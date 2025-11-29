%% EE5101 Task 1: Pole Placement
% Matriculation: 8246
clc; clear; close all;

% 1. Define System (Corrected Matrices)
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

% 2. Design Configurations
% Design A: Moderate/Standard (Meets specs safely)
P_A = [-2, -3, -4, -5]; 
K_A = place(A, B, P_A);

% Design B: Aggressive (Faster but expensive control)
P_B = [-5, -6, -8, -10];
K_B = place(A, B, P_B);

% 3. Create Closed-Loop Systems
sys_cl_A = ss(A - B*K_A, B, C, D); % Note: Input is reference
sys_cl_B = ss(A - B*K_B, B, C, D);

% 4. Simulation 1: Initial Condition Response (Zero Input)
x0 = [0.5; -0.1; 0.3; -0.8];
t = 0:0.01:5;
[y_init_A, t, x_init_A] = initial(sys_cl_A, x0, t);
[y_init_B, t, x_init_B] = initial(sys_cl_B, x0, t);

% Calculate Control Efforts for Initial Response (u = -Kx)
u_A = (-K_A * x_init_A')';
u_B = (-K_B * x_init_B')';

% 5. Simulation 2: Step Response (Reference Tracking)
% Pole placement alone doesn't guarantee zero steady state error 
% for tracking, but we check transient specs (Overshoot/Settling time).
opt = stepDataOptions('StepAmplitude', 1);

% 6. Plotting for Report
% Figure 4.1: Initial State Response
figure('Name', 'Fig 4.1: Initial State Response');
subplot(2,2,1); plot(t, x_init_A(:,1), 'b', t, x_init_B(:,1), 'r--'); title('State x_1'); grid on; legend('Design A', 'Design B');
subplot(2,2,2); plot(t, x_init_A(:,2), 'b', t, x_init_B(:,2), 'r--'); title('State x_2'); grid on;
subplot(2,2,3); plot(t, x_init_A(:,3), 'b', t, x_init_B(:,3), 'r--'); title('State x_3'); grid on;
subplot(2,2,4); plot(t, x_init_A(:,4), 'b', t, x_init_B(:,4), 'r--'); title('State x_4'); grid on;

% Figure 4.2: Control Effort
figure('Name', 'Fig 4.2: Control Effort');
subplot(2,1,1); plot(t, u_A(:,1), 'b', t, u_B(:,1), 'r--'); title('Control u_1 (VGT)'); ylabel('V'); grid on; legend('Design A', 'Design B');
subplot(2,1,2); plot(t, u_A(:,2), 'b', t, u_B(:,2), 'r--'); title('Control u_2 (EGR)'); ylabel('V'); grid on;

% Figure 4.3: Step Response Check
figure('Name', 'Fig 4.3: Step Response Check');
step(sys_cl_A);
title('Step Response Verification (Design A)');
grid on;

% Output Matrices for Report
fprintf('Gain Matrix K_A (Moderate):\n'); disp(K_A);
fprintf('Gain Matrix K_B (Aggressive):\n'); disp(K_B);