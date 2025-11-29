%% Task 2: LQR Control
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

% 2. LQR Design Configurations
% Design A: Performance-Focused (Cheap Control)
% High Q, Low R -> Fast response, High energy
Q_A = 1000 * eye(4); 
R_A = 1 * eye(2);
[K_LQR_A, ~, ~] = lqr(A, B, Q_A, R_A);

% Design B: Efficiency-Focused (Expensive Control)
% Same Q, Higher R -> Slower response, Low energy
Q_B = 1000 * eye(4);
R_B = 100 * eye(2); 
[K_LQR_B, ~, ~] = lqr(A, B, Q_B, R_B);

% 3. Create Closed-Loop Systems
sys_cl_A = ss(A - B*K_LQR_A, B, C, D); 
sys_cl_B = ss(A - B*K_LQR_B, B, C, D);

% 4. Simulation 1: Initial Condition Response (Zero Input)
x0 = [0.5; -0.1; 0.3; -0.8];
t = 0:0.01:5;
[y_init_A, t, x_init_A] = initial(sys_cl_A, x0, t);
[y_init_B, t, x_init_B] = initial(sys_cl_B, x0, t);

% Calculate Control Efforts for Initial Response (u = -Kx)
u_A = (-K_LQR_A * x_init_A')';
u_B = (-K_LQR_B * x_init_B')';

% 5. Simulation 2: Step Response (Verification for Design B)
% verify Design B because it is slower; if it passes, A surely passes.
opt = stepDataOptions('StepAmplitude', 1);

% 6. Plotting for Report
% Figure 5.1: Initial State Response Comparison
figure('Name', 'Fig 5.1: LQR State Response');
subplot(2,2,1); plot(t, x_init_A(:,1), 'b', t, x_init_B(:,1), 'r--'); title('State x_1'); grid on; legend('Design A (Fast)', 'Design B (Efficient)');
subplot(2,2,2); plot(t, x_init_A(:,2), 'b', t, x_init_B(:,2), 'r--'); title('State x_2'); grid on;
subplot(2,2,3); plot(t, x_init_A(:,3), 'b', t, x_init_B(:,3), 'r--'); title('State x_3'); grid on;
subplot(2,2,4); plot(t, x_init_A(:,4), 'b', t, x_init_B(:,4), 'r--'); title('State x_4'); grid on;

% Figure 5.2: Control Effort Comparison
figure('Name', 'Fig 5.2: LQR Control Effort');
subplot(2,1,1); plot(t, u_A(:,1), 'b', t, u_B(:,1), 'r--'); title('Control u_1 (VGT)'); ylabel('V'); grid on; legend('Design A', 'Design B');
subplot(2,1,2); plot(t, u_A(:,2), 'b', t, u_B(:,2), 'r--'); title('Control u_2 (EGR)'); ylabel('V'); grid on;

% Figure 5.3: Step Response Check (Design B)
figure('Name', 'Fig 5.3: LQR Step Response (Design B)');
step(sys_cl_B);
title('Step Response Verification (LQR Design B)');
grid on;

% Output Matrices for Report
fprintf('LQR Gain Matrix K (Design A - Cheap Control):\n'); disp(K_LQR_A);
fprintf('LQR Gain Matrix K (Design B - Expensive Control):\n'); disp(K_LQR_B);