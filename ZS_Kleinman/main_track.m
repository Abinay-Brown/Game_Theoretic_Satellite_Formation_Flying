clear;
clc;
close all;
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

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     (5*c^2 - 2)*(w^2), 0, 0, 0, 2*w*c, 0;
     0, 0, 0, 2*w*c, 0, 0;
     0, 0, -(k^2), 0, 0, 0];

B1 = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1, 0, 0;
     0, 1, 0;
     0, 0, 1];

B2 = B1;

Abar = [A, zeros(6, 3);
        -eye(3), zeros(3, 6)];
B1bar = [B1; zeros(3, 3)];
B2bar = B1bar;
Cbar = eye(9);
Dbar = eye(3);
Qbar = Cbar'*Cbar;
Rbar = Dbar'*Dbar;

% C = eye(6);
% D = eye(3);
% Q = C'*C;
% R = D'*D;

epsilon = 0.0005;
tol = 1e-7; 
% P = GARE(A, B1, B2, Q, R, epsilon, tol);

P = GARE(Abar, B1bar, B2bar, Qbar, Rbar, epsilon, tol);

tspan = 0:0.1:30;
x0 = [50, 30, -25, 1, 1, 1, 0, 0, 0];
params.A = Abar;
params.B1 = B1bar;
params.B2 = B2bar;
params.R = Rbar;
params.P = P;
params.y = 100;
EOM = @(t, x) dynamics(t, x, params);
options = odeset('RelTol', 1e-9, 'AbsTol',1e-9);
% options = odeset('OutputFcn',@odeplot);
[t, sol] = ode45(EOM, tspan, x0, options);

% Create the figure
figure;

% Subplot 1: Position
subplot(2, 1, 1);
plot(t, 10*ones(size(t)), '-g', 'LineWidth', 1.5); hold on;
plot(t, sol(:, 1), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t, sol(:, 2), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t, sol(:, 3), '-b', 'LineWidth', 1.5);          % Black solid line

xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Position (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'Reference','$x$', '$y$', '$z$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Subplot 2: Velocity
subplot(2, 1, 2);
plot(t, sol(:, 4), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t, sol(:, 5), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t, sol(:, 6), '-b', 'LineWidth', 1.5);          % Black solid line
xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Velocity (m/s)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$V_x$', '$V_y$', '$V_z$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Additional AIAA Style Formatting
set(gcf, 'Color', 'w'); % Set figure background to white


u_sol = zeros(3, length(t));
d_sol = zeros(3, length(t));
for i = 1: length(t)
    u_sol(:, i) = -inv(Rbar)*B1bar'*P*sol(i, :)';
    d_sol(:, i) =  (1/params.y^2)*B2bar'*P*sol(i, :)';
end
figure(2);

% Subplot 1: Position
subplot(2, 1, 1);
plot(t, u_sol(1, :), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t, u_sol(2, :), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t, u_sol(3, :), '-b', 'LineWidth', 1.5);          % Black solid line

xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Chaser Control Input U (m/s^2)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$Ux$', '$Uy$', '$Uz$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Subplot 2: Disturbance
subplot(2, 1, 2);
plot(t, d_sol(1, :), '.r', 'LineWidth', 1.5); hold on; % Black dashed line
plot(t, d_sol(2, :), ':k', 'LineWidth', 1.5);          % Black dotted line
plot(t, d_sol(3, :), '-b', 'LineWidth', 1.5);          % Black solid line
xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Disturbance Input D (m/s^2)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$D_x$', '$D_y$', '$D_z$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Additional AIAA Style Formatting
set(gcf, 'Color', 'w'); % Set figure background to white


function xdot = dynamics(t, state, params)
    x1 = state(1);
    x2 = state(2);
    x3 = state(3);
    x4 = state(4);
    x5 = state(5);
    x6 = state(6);
    
    x = [x1, x2, x3, x4, x5, x6, 10-x1, 10-x2, 10-x3]';
    A = params.A;
    B1 = params.B1;
    B2 = params.B2;
   
    R = params.R;
    P = params.P;
    y = params.y;
    u = -inv(R)*B1'*P*state;
    v = (1/y^2)*B2'*P*state;
    xdot = A*x + B1*u + B2*v;
    xdot(7) = 10-x1;
    xdot(8) = 10-x2;
    xdot(9) = 10-x3;
end

function P = GARE(A, B1, B2, Q, R, epsilon, tol)
    max_iter = 100; % Maximum iterations
    iter = 0;
    P = Q;
    while true
        A_tilde = A - B2 * inv(R) * B2' * P - epsilon*eye(size(A,1));
        P_next = lyap(A_tilde', Q + P * B2 * inv(R) * B2' * P);
        if norm(P_next - P, 'fro') < tol
            break;
        end
        P = P_next;
        iter = iter + 1;
    
        if iter > max_iter
            error('Kleinman method did not converge');
        end
    end      
end