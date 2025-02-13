clear;
clc;
close all;
%% Parameters
J2 = 1.08263 * 10^-3;
Re = 6378 * 10^3;
rc = Re + 700*10^3;
ic = 98 * (pi/180);
mu = 3.986 * 10^14;
T = 2 * pi * sqrt((rc^3)/mu);
wc = 2 * pi / T;
w =  2 * pi / T;

s = 2*J2*(Re^2)/(8*rc^2)*(1 + 3*cos(2*ic));
c = sqrt(1 + s);
k = wc*sqrt(1+s) + 3*J2*wc*(Re^2)/(2*rc^2)*cos(ic);
params.s = s;
params.c = c;
params.k = k;
params.w = w;
params.gamma = 100;

tspan = 0:0.1:20;
W1 = 10*rand(1, 45);
W2 = 100*rand(1, 45);
W3 = 20*rand(1, 45);
x0 = [100, 100, 100, 12, -10, 20, 0, 0, 0, W1, W2, W3];

% out = reldynamics(0, x0, params);
EOM = @(t, x) NWreldynamics(t, x, params);
options = odeset('OutputFcn',@odeplot, 'RelTol', 1e-9, 'AbsTol',1e-9);
% options = odeset('OutputFcn',@odeplot);
[t, sol] = ode45(EOM, tspan, x0, options);

% plot3(sol(:, 1), sol(:, 2), sol(:, 3));

% subplot(6, 1, 1);
% plot(t, sol(:, 1));
% xlabel('time (sec)');
% ylabel('x (m)');
% 
% subplot(6, 1, 2);
% plot(t, sol(:, 2));
% xlabel('time (sec)');
% ylabel('y (m)');
% 
% subplot(6, 1, 3)
% plot(t, sol(:, 3));
% xlabel('time (sec)');
% ylabel('z (m)');
% 
% subplot(6, 1, 4)
% plot(t, sol(:, 4));
% xlabel('time (sec)');
% ylabel('Vx (m/s)');
% 
% subplot(6, 1, 5)
% plot(t, sol(:, 5));
% xlabel('time (sec)');
% ylabel('Vy (m/s)');
% 
% subplot(6, 1, 6)
% plot(t, sol(:, 6));
% xlabel('time (sec)');
% ylabel('Vz (m/s)');
