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

tspan = 0:0.1:1500;
W1 = 5*ones(1, 21);
W2 = 4*ones(1, 21);
W3 = 3*ones(1, 21);
x0 = [50, 50, 50, 1.5, 1.5, 1.5, W1, W2, W3];

% out = reldynamics(0, x0, params);
EOM = @(t, x) reldynamics(t, x, params);
options = odeset('OutputFcn',@odeplot, 'RelTol', 1e-9, 'AbsTol',1e-9);
% options = odeset('OutputFcn',@odeplot);
[t, sol] = ode45(EOM, tspan, x0, options);

% plot3(sol(:, 1), sol(:, 2), sol(:, 3));

% Create the figure
% Subplot 1: Position
subplot(2, 1, 1);
plot(t(1:end), sol(1:end, 1), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t(1:end), sol(1:end, 2), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t(1:end), sol(1:end, 3), '-b', 'LineWidth', 1.5);          % Black solid line

xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Position (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$x$', '$y$', '$z$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Subplot 2: Velocity
subplot(2, 1, 2);
plot(t(1:end), sol(1:end, 4), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t(1:end), sol(1:end, 5), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t(1:end), sol(1:end, 6), '-b', 'LineWidth', 1.5);          % Black solid line
xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Velocity (m/s)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$V_x$', '$V_y$', '$V_z$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Additional AIAA Style Formatting
set(gcf, 'Color', 'w'); % Set figure background to white
save('Solution.mat')