addpath(fullfile('..', 'src'));

close all
clear all
clc

%% 
Ts = 1/20;
rocket = Rocket(Ts);
Tf = 10;
H = 7;
%% Linearization around trim point
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
%% Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);
%%  3.1 Design MPC controller X
mpc_x = MpcControl_x(sys_x, Ts, H);
x_x = [0 0 0 3]'; % (wy, beta, vx, x) Initial state    
% Open−loop trajectory
[~, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); 
%% Receiding horizon approach for computing the closed loop trajectory:
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% Design MPC controller Y
mpc_y = MpcControl_y(sys_y, Ts, H);

x_y = [0 0 0 3]'; % (wx, alpha, vy, y) Initial state


% Open−loop trajectory
[~, T_opt, Y_opt, U_opt] = mpc_y.get_u(x_y);
U_opt(:,end+1) = NaN;

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Y_opt, U_opt, sys_y, xs, us); 

%% Receiding horizon approach for computing the closed loop trajectory:
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);

%% Design MPC controller Z

mpc_z = MpcControl_z(sys_z, Ts, H);

x_z = [0 3]'; % (vz, z) Initial state


% Open−loop trajectory,
[~, T_opt, Z_opt, U_opt] = mpc_z.get_u(x_z);
U_opt(:,end+1) = NaN;
U_opt = U_opt+us(3)*ones(1,size(U_opt,2));

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Z_opt, U_opt, sys_z, xs, us); 

%% Receiding horizon approach for computing the closed loop trajectory:
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, x_z, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us);


%% Design MPC controller roll
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
x_roll = [0 deg2rad(30)]'; % (wz, gamma) Initial state


% Open−loop trajectory,
[u, T_opt, Gamma_opt, U_opt] = mpc_roll.get_u(x_roll);
U_opt(:,end+1) = NaN;

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Gamma_opt, U_opt, sys_roll, xs, us); 
%% Receiding horizon approach for computing the closed loop trajectory:
[T, Gamma_sub, U_sub] = rocket.simulate_f(sys_roll, x_roll, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, Gamma_sub, U_sub, sys_roll, xs, us);