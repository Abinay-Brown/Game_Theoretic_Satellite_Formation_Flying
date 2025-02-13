clear;
clc;
close all;
load("Solution.mat")

%% State Plots
% subplot(3, 2, 1);
% plot(t, sol(:, 1), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('x (m)');
% xlim([0, 10900])
% 
% 
% subplot(3, 2, 3);
% plot(t, sol(:, 2), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('y (m)');
% xlim([0, 10900])
% 
% 
% subplot(3, 2, 5)
% plot(t, sol(:, 3), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('z (m)');
% xlim([0, 10900])
% 
% 
% subplot(3, 2, 2)
% plot(t, sol(:, 4), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('Vx (m/s)');
% xlim([0, 10900])
% 
% 
% subplot(3, 2, 4)
% plot(t, sol(:, 5), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('Vy (m/s)');
% xlim([0, 10900])
% 
% subplot(3, 2, 6)
% plot(t, sol(:, 6), 'LineWidth', 1.2);
% xlabel('time (sec)');
% ylabel('Vz (m/s)');
% xlim([0, 10900])

u3_sol = zeros(3, length(t));
d4_sol = zeros(3, length(t));

for i = 1:length(sol)

    x1 = sol(i, 1);
    x2 = sol(i, 2);
    x3 = sol(i, 3);
    
    x4 = sol(i, 4);
    x5 = sol(i, 5);
    x6 = sol(i, 6);
    
    x = [x1, x2, x3, x4, x5, x6]'; % state vector
    
    W1 = sol(i, 7:27)';  % Player 1 Critic Weights
    W2 = sol(i, 28:48)'; % Player 1 Actor Weights
    W3 = sol(i, 49:69)'; % Player 2 Critic Weights
    W4 = sol(i, 70:90)'; % Player 2 Actor Weights
    grad_phi1 = [2*x1,    0,    0,   0,     0,   0;
                   x2,   x1,    0,    0,    0,   0;
                   x3,    0,   x1,    0,    0,   0;
                   x4,    0,    0,   x1,    0,   0;
                   x5,    0,    0,    0,   x1,   0;
                   x6,    0,    0,    0,    0,  x1;
                    0, 2*x2,    0,    0,    0,   0;
                    0,   x3,   x2,    0,    0,   0;
                    0,   x4,    0,   x2,    0,   0;
                    0,   x5,    0,    0,   x2,   0;
                    0,   x6,    0,    0,    0,  x2;
                    0,    0, 2*x3,    0,    0,   0;
                    0,    0,   x4,   x3,    0,   0;
                    0,    0,   x5,    0,   x3,   0;
                    0,    0,   x6,    0,    0,  x3;
                    0,    0,    0, 2*x4,    0,   0;
                    0,    0,    0,   x5,   x4,   0;
                    0,    0,    0,   x6,    0,  x4;
                    0,    0,    0,    0, 2*x5,   0;
                    0,    0,    0,    0,   x6,  x5;
                    0,    0,    0,    0,   0, 2*x6];
    gm = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1];
        
    km = [0, 0, 0;
          0, 0, 0;
          0, 0, 0;
          1, 0, 0;
          0, 1, 0;
          0, 0, 1];
    
    grad_phi2 = grad_phi1;
    R22 = eye(3);
    R21 = eye(3);
    R11 = 2*R22;
    R12 = 2*R21;
    u3 = -(1/2) * inv(R11) * gm' * grad_phi1' * W3;
    d4 = -(1/2) * inv(R22) * km' * grad_phi2' * W4;
    
    u3_sol(:, i) = u3;
    d4_sol(:, i) = d4;
    disp(i);
end


% Create the figure
figure;

% Subplot 1: Position
subplot(2, 1, 1);
plot(t(1:1500), u3_sol(1, 1:1500), '.r', 'LineWidth', 1.5); 
hold on; % Black dashed line
plot(t(1:1500), u3_sol(2, 1:1500), ':k', 'LineWidth', 1.5);
hold on
plot(t(1:1500), u3_sol(3, 1:1500), '-b', 'LineWidth', 1.5);          % Black solid line
xlim([0, 1200])
xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Chaser Control Input (m/s^2)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$Ux_1$', '$Uy_1$', '$Uz_1$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box

% Subplot 2: Velocity
subplot(2, 1, 2);
plot(t(1:1500), d4_sol(1, 1:1500), '.r', 'LineWidth', 1.5); 
hold on; % Black dashed line
plot(t(1:1500), d4_sol(2, 1:1500), ':k', 'LineWidth', 1.5); 

hold on;
plot(t(1:1500), d4_sol(3, 1:1500), '-b', 'LineWidth', 1.5);          % Black solid line
xlabel('Time (sec)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('Target Control Input (m/s^2)', 'FontSize', 12, 'FontName', 'Times New Roman');
legend({'$Ux_2$', '$Uy_2$', '$Uz_2$'}, 'Interpreter', 'latex', 'FontSize', 10, 'Location', 'northeast', 'Box', 'off');
box on; % Enable thick outline
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1); % Thicker bounding box
xlim([0, 1200])
% Additional AIAA Style Formatting
set(gcf, 'Color', 'w'); % Set figure background to white

