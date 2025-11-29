% EE5101/ME5401
% Model Initialization
% My Student Matriculation Last 4 Digits: 8246

clc; clear; close all;

%% 1. Define Parameters
a = 8;
b = 2;
c = 4;
d = 6;

fprintf('Student Parameters: a=%d, b=%d, c=%d, d=%d\n', a, b, c, d);

%% 2. Construct System Matrices
% Matrix A (4x4)
A = zeros(4,4);
A(1,1) = -8.8487+(a-b)/5;
A(1,2) = -0.0399;       
A(1,3) = -5.5500+(c+d)/10;
A(1,4) = 3.5846;

A(2,1) = -4.5740;
A(2,2) = 2.5010*(d+5)/(c+5);
A(2,3) = -4.3662;
A(2,4) = -1.1183-(a-c)/20;

A(3,1) = 3.7698;
A(3,2) = 16.1212-c/5;
A(3,3) = -18.2103+(a+d)/(b+4);              
A(3,4) = 4.4936;      

A(4,1) = -8.5645-(a-b)/(c+d+2);
A(4,2) = 8.3742;
A(4,3) = -4.4331;
A(4,4) = -7.7181*(c+5)/(b+5);

% Matrix B (4x2)
B = zeros(4,2);
B(1,1) = 0.0564+b/(10+c);
B(1,2) = 0.0319;

B(2,1) = 0.0165-(c+d-5)/(1000+20*a); 
B(2,2) = -0.02;

B(3,1) = 4.4939;
B(3,2) = 1.5985*(a+10)/(b+12);

B(4,1) = -1.4269;
B(4,2) = -0.2730;

% Matrix C (2x4)
C = zeros(2,4);
C(1,1) = -3.2988;
C(1,2) = -2.1932+(10*c+d)/(100+5*a);
C(1,3) = 0.0370;
C(1,4) = -0.0109;

C(2,1) = 0.2922-(a*b)/500;
C(2,2) = -2.1506;
C(2,3) = -0.0104;
C(2,4) = 0.0163;

% Matrix D (Assuming zero as it's not explicitly given and standard for this type)
D = zeros(2,2);

%% 3. Create State Space System
sys = ss(A, B, C, D);

%% 4. Initial Analysis (Open Loop)
fprintf('\n--- System Matrices ---\n');
disp('A = '); disp(A);
disp('B = '); disp(B);
disp('C = '); disp(C);

fprintf('\n--- Open Loop Eigenvalues (Poles) ---\n');
poles = eig(A);
disp(poles);

if all(real(poles) < 0)
    disp('Result: The open-loop system is STABLE.');
else
    disp('Result: The open-loop system is UNSTABLE.');
end

%% 5. Step Response Test
figure;
step(sys);
title('Open Loop Step Response for Matric No: ...8246');
grid on;