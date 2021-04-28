clear all;
close all;
clc;
%588871
% Double pendulum kinematic analysis
%% Coordinates
% ground
q1 = [0; 0; 0];
% link 1
q2 = [1
    0
    0];
% link 2
q3 = [2+cosd(35)
    sind(35)
    deg2rad(35)];

q_0 = [q1; q2; q3]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and link 1
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-1; 0];

% 2 connects link 1 and link 2
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [1; 0];
revolute(2).s_j = [-1; 0];

% % Check revolute joint constraints
% r = revolute(3);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

%% Simple constraints

% two simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

% % check simple constraints
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) -7*pi/36 - 2.3 * t;
driving.d_k_t = @(t) -2.3;
driving.d_k_tt = @(t) 0;
% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using fsolve
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
tic
[T, Q] = position_fsolve(C_fun, 1, q_0, 0.1);
toc
%% Jacobian of our constraints
Cq = constraint_dq(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using NR
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
tic
[T, Q] = position_NR(C_fun, Cq_fun, 1, q_0, 0.1);
toc
%% Verify Ct
Ct = constraint_dt(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using NR 
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
%acceleration
Ctt_fun= @(t, q, dq) constraint_ddt(revolute, simple, driving, t, q, dq);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, Ctt_fun, 1, q_0, 0.1);

%% Some verification plots
figure
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal 
legend('Link 1','Link 2','Origin')
title('Position Analysis for Double pendulum')
%% Some verification plots
figure
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
legend('Link 1','Link 2','Origin')
title('Velocity Analysis for Double pendulum')
%% Some verification plots
figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    0, 0, '*', 'LineWidth', 2);
axis equal 
legend('Link 1','Link 2','Origin')
title('Acceleration Analysis for Double pendulum')
