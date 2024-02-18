addpath(fullfile('..', 'src'));

close all
clear all
clc

%% This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);
rocket.delay = 0;
H = 3; % Horizon length in seconds

%% Trial with constant reference
nmpc = NmpcControl(rocket, H);
ref = [0.5, 0, 1, deg2rad(5)]';

x = zeros(12,1);

Tf = 10;
[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);


%% Non linear MPC

Tf =30;
% TVC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);
[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
%%
% TVC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
%% Comparison with linear MPC
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
x = zeros(12,1);
Tf =30;
%Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 7; % Horizon length in seconds

mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% TVC reference with specified maximum roll = 15 deg
roll_max = deg2rad(15);
ref = @(t_, x_) ref_TVC(t_, roll_max);

% Simulate
[T, X, U, Ref] = rocket.simulate(x, Tf, @mpc.get_u, ref);
% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);

%%
% TVC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

% Simulate
[T, X, U, Ref] = rocket.simulate(x, Tf, @mpc.get_u, ref);
% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);



