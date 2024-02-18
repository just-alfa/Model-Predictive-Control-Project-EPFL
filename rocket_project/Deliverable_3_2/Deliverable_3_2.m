addpath(fullfile('..', 'src'));

close all
clear all
clc

%% This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);
H = 7;
Tf = 10;
% Linearization around trim point
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
% Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% MPC for tracking x
x_x = [0 0 0 3]';
pos_ref =-4;
mpc_x = MpcControl_x(sys_x, Ts, H);
[~, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); 
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, pos_ref);

%% MPC for tracking y
x_y = [0 0 0 3]';
pos_ref =-4;
mpc_y = MpcControl_y(sys_y, Ts, H);
[~, T_opt, Y_opt, U_opt] = mpc_y.get_u(x_y);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, Y_opt, U_opt, sys_y, xs, us); 
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, pos_ref);

%% MPC for tracking z
x_z = [0 3]';
pos_ref =-4;
mpc_z = MpcControl_z(sys_z, Ts, H);
[~, T_opt, Z_opt, U_opt] = mpc_z.get_u(x_z);
U_opt(:,end+1) = NaN;
U_opt = U_opt+us(3)*ones(1,size(U_opt,2));
ph = rocket.plotvis_sub(T_opt, Z_opt, U_opt, sys_z, xs, us); 
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, x_z, Tf, @mpc_z.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, pos_ref);

%% MPC for tracking roll
x_roll = [0 deg2rad(40)]';
pos_ref =deg2rad(35);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
[u, T_opt, Gamma_opt, U_opt] = mpc_roll.get_u(x_roll);
U_opt(:,end+1) = NaN;
ph = rocket.plotvis_sub(T_opt, Gamma_opt, U_opt, sys_roll, xs, us); 
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x_roll, Tf, @mpc_roll.get_u, pos_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, pos_ref);



