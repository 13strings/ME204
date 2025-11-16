%% Maximum Bending Moment Calculator for Crank Link
% Standalone script to find worst-case bending on crank (l2)
clear; clc; close all;

%% Input Parameters (from your optimization results)
% Mechanism geometry
l2 = 20e-3;      % m, crank length
l3 = 130e-3;  % m, connecting rod length
e = 21e-3;   % m, offset distance

% Spring parameters
PRELOAD = 5.6;           % lbf
SPRING_CONSTANT = 11.9;  % lbf/in
PULLEY_RATIO = 1/1;      % Force reduction ratio

% Convert to SI
K_SPRING = SPRING_CONSTANT * 175.127;  % N/m
F_PRELOAD = PRELOAD * 4.44822;         % N

%% Simulate Over Full Rotation
n_points = 360;
theta2_range = linspace(0, 2*pi, n_points);

F_connect = zeros(n_points, 1);
F_slider = zeros(n_points, 1);
theta3_range = zeros(n_points, 1);

for i = 1:n_points
    theta2 = theta2_range(i);
    
    % Calculate theta3
    theta3 = calculate_theta3(l2, l3, e, theta2);
    theta3_range(i) = theta3;
    
    % Calculate slider displacement
    displacement = calculate_slider_position(l2, l3, e, theta2);
    
    % Spring force at slider
    F_s = spring_force(displacement, F_PRELOAD, K_SPRING, PULLEY_RATIO);
    F_slider(i) = F_s;
    
    % Force in connecting rod (causes bending in crank)
    if abs(cos(theta3)) > 1e-6
        F_connect(i) = abs(F_s / cos(theta3));
    else
        F_connect(i) = 0;  % Toggle position
    end
end

%% Find Maximum Bending Moment
[F_max, idx_max] = max(F_connect);
theta2_worst = theta2_range(idx_max);
theta3_worst = theta3_range(idx_max);
F_slider_worst = F_slider(idx_max);

% Bending moment at crank root (cantilever beam)
M_bending_max = F_max * l2;

%% Display Results
fprintf('================================================================\n');
fprintf('MAXIMUM BENDING MOMENT ON CRANK LINK (l2)\n');
fprintf('================================================================\n');
fprintf('Mechanism Geometry:\n');
fprintf('  Crank length (l2):          %.2f mm\n', l2*1000);
fprintf('  Connecting rod (l3):        %.2f mm\n', l3*1000);
fprintf('  Offset (e):                 %.2f mm\n\n', e*1000);

fprintf('Worst-Case Loading Condition:\n');
fprintf('  Crank angle (θ₂):           %.1f°\n', rad2deg(theta2_worst));
fprintf('  Connecting rod angle (θ₃):  %.1f°\n', rad2deg(theta3_worst));
fprintf('  Spring force at slider:     %.2f N (%.2f lbf)\n', F_slider_worst, F_slider_worst/4.44822);
fprintf('  Force in connecting rod:    %.2f N (%.2f lbf)\n\n', F_max, F_max/4.44822);

fprintf('Maximum Bending Moment:\n');
fprintf('  M_bending = %.6f Nm\n', M_bending_max);
fprintf('  M_bending = %.4f N-mm\n', M_bending_max*1000);
fprintf('================================================================\n\n');

fprintf('>>> USE THIS VALUE IN SHAFT SIZING CODE:\n');
fprintf('    M_b = %.6f;  %% Nm\n\n', M_bending_max);

%% Plot Results
figure('Position', [100, 100, 1200, 500]);

% Force in connecting rod vs crank angle
subplot(1, 2, 1);
plot(rad2deg(theta2_range), F_connect, 'b-', 'LineWidth', 2);
hold on;
plot(rad2deg(theta2_worst), F_max, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Crank Angle θ₂ (degrees)', 'FontSize', 12);
ylabel('Force in Connecting Rod (N)', 'FontSize', 12);
title('Connecting Rod Force vs Crank Angle', 'FontSize', 13, 'FontWeight', 'bold');
legend('F_{connect}', sprintf('Max = %.1f N at θ₂=%.1f°', F_max, rad2deg(theta2_worst)), 'Location', 'best');
grid on;

% Bending moment vs crank angle
subplot(1, 2, 2);
M_bending = F_connect * l2;
plot(rad2deg(theta2_range), M_bending, 'r-', 'LineWidth', 2);
hold on;
plot(rad2deg(theta2_worst), M_bending_max, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Crank Angle θ₂ (degrees)', 'FontSize', 12);
ylabel('Bending Moment (Nm)', 'FontSize', 12);
title('Bending Moment at Crank Root vs Crank Angle', 'FontSize', 13, 'FontWeight', 'bold');
legend('M_{bending}', sprintf('Max = %.4f Nm at θ₂=%.1f°', M_bending_max, rad2deg(theta2_worst)), 'Location', 'best');
grid on;

sgtitle(sprintf('Crank Link Analysis: l2=%.1fmm, l3=%.1fmm, e=%.1fmm', ...
    l2*1000, l3*1000, e*1000), 'FontSize', 14, 'FontWeight', 'bold');

%% Helper Functions
function F = spring_force(displacement, F_preload, K_spring, pulley_ratio)
    if displacement <= 0
        F = 0;
        return;
    end
    
    disp_in = displacement * 39.3701;  % m to inches
    force_lbf = (F_preload/4.44822) + (K_spring/175.127) * disp_in;
    F = force_lbf * 4.44822 * pulley_ratio;  % Convert to N and apply pulley ratio
end

function theta3 = calculate_theta3(l2, l3, e, theta2)
    l2_sin_theta2 = l2 * sin(theta2);
    arg = (e - l2_sin_theta2) / l3;
    
    if abs(arg) > 1
        theta3 = NaN;
        return;
    end
    
    theta3 = asin(arg);
end

function displacement = calculate_slider_position(l2, l3, e, theta2)
    theta3 = calculate_theta3(l2, l3, e, theta2);
    if isnan(theta3)
        displacement = NaN;
        return;
    end
    
    % Current position
    x_current = l2 * cos(theta2) + l3 * cos(theta3);
    
    % Extended dead center position (theta2 = 0)
    theta3_extended = calculate_theta3(l2, l3, e, 0);
    x_extended = l2 * cos(0) + l3 * cos(theta3_extended);
    
    % Displacement from extended position
    displacement = x_extended - x_current;
end