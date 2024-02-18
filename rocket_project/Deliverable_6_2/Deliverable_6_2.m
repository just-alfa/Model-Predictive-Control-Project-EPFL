addpath(fullfile('..', 'src'));

close all
clear all
clc

%%  This file should produce all the plots for the deliverable

Ts = 1/40; % Higher sampling rate for this part!

rocket = Rocket(Ts);
... Define NMPC ...
H = 1; % Horizon length in seconds
expected_delay = 4;
nmpc = NmpcControl(rocket, H, expected_delay);
x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 2.5;
rocket.mass = 1.75;
rocket.delay = 6; % 0 if not specified
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);