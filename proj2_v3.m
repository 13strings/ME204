%% Offset Slider-Crank Optimization Script
% Optimizes l2, l3, and e for minimum torque while meeting stroke requirement

clear; clc; close all;

%% Constants
global STROKE PRELOAD SPRING_CONSTANT PULLEY_RATIO K_SPRING F_PRELOAD

STROKE =40e-3;          % 40mm in meters
PRELOAD = 5.6;           % lbf
SPRING_CONSTANT = 11.9;  % lbf/in
PULLEY_RATIO = 1/3;      % Force reduction from pulley system

% Convert to SI units
K_SPRING = SPRING_CONSTANT * 175.127;  % N/m (lbf/in to N/m)
F_PRELOAD = PRELOAD * 4.44822;         % N

fprintf('=============================================================\n');
fprintf('Offset Slider-Crank Optimization\n');
fprintf('=============================================================\n');
fprintf('Target stroke:      %.1f mm\n', STROKE*1000);
fprintf('Spring preload:     %.1f lbf\n', PRELOAD);
fprintf('Spring constant:    %.1f lbf/in\n', SPRING_CONSTANT);
fprintf('Force reduction:    %.2f (pulley ratio)\n', PULLEY_RATIO);
fprintf('=============================================================\n\n');

%% Optimization Setup
% Decision variables: [l2, l3, e]
% Bounds (in meters)
lb = [0.010, 0.020, 0.015];  % Lower bounds [l2, l3, e]
ub = [0.150, 0.15, 0.150];  % Upper bounds

% Initial guess
x0 = [0.030, 0.150, 0.050];

% Optimization options
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxIterations', 1000, ...
    'MaxFunctionEvaluations', 10000, ...
    'ConstraintTolerance', 1e-8, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-10);

% Run optimization
fprintf('Starting optimization...\n\n');
[x_opt, fval, exitflag, output] = fmincon(@objective_function, x0, ...
    [], [], [], [], lb, ub, @nonlinear_constraints, options);

%% Extract Results
l2_opt = x_opt(1);
l3_opt = x_opt(2);
e_opt = x_opt(3);

%% Display Results
fprintf('\n=============================================================\n');
fprintf('OPTIMIZATION RESULTS\n');
fprintf('=============================================================\n');
fprintf('Crank length (l2):        %.2f mm\n', l2_opt*1000);
fprintf('Connecting rod (l3):      %.2f mm\n', l3_opt*1000);
fprintf('Offset distance (e):      %.2f mm\n', e_opt*1000);

% Calculate actual stroke from geometry
x_extended = l2_opt + sqrt(l3_opt^2 - e_opt^2);
x_contracted = -l2_opt + sqrt(l3_opt^2 - e_opt^2);
stroke_actual = x_extended - x_contracted;

fprintf('\nStroke from constraint:   %.4f mm\n', (2*sqrt(l3_opt^2-e_opt^2)-2*l2_opt)*1000);
fprintf('Stroke from geometry:     %.4f mm (target: %.1f mm)\n', stroke_actual*1000, STROKE*1000);
fprintf('Constraint e > l2:        %s (%.2fx)\n', ...
    string(e_opt > l2_opt), e_opt/l2_opt);

%% Analyze Optimal Design
n_points = 360;
[theta2_range, torques, displacements, forces] = simulate_mechanism(l2_opt, l3_opt, e_opt, n_points);

fprintf('\nPeak torque:              %.4f Nm (%.2f oz-in)\n', max(torques), max(torques)*141.612);
fprintf('Average torque:           %.4f Nm (%.2f oz-in)\n', mean(torques), mean(torques)*141.612);
fprintf('RMS torque:               %.4f Nm (%.2f oz-in)\n', rms(torques), rms(torques)*141.612);
fprintf('Min torque:               %.4f Nm (%.2f oz-in)\n', min(torques), min(torques)*141.612);
fprintf('\nMax displacement:         %.2f mm (at theta2 = %.1f deg)\n', ...
    max(displacements)*1000, rad2deg(theta2_range(displacements == max(displacements))));
fprintf('Min displacement:         %.2f mm (at theta2 = %.1f deg)\n', ...
    min(displacements)*1000, rad2deg(theta2_range(displacements == min(displacements))));
fprintf('Displacement range:       %.2f mm\n', (max(displacements)-min(displacements))*1000);
fprintf('\nMax spring force:         %.2f lbf\n', max(forces)/4.44822);
fprintf('Force at slider (max):    %.2f lbf (with pulley)\n', max(forces)/4.44822);
fprintf('=============================================================\n');

%% Plot Results
plot_results(l2_opt, l3_opt, e_opt, theta2_range, torques, displacements, forces);

%% Theta3 Analysis
plot_theta3_analysis(l2_opt, l3_opt, e_opt);

%% ========================================================================
% HELPER FUNCTIONS
%% ========================================================================

function F = spring_force(displacement)
    % Calculate force at slider (accounting for pulley ratio)
    global PRELOAD SPRING_CONSTANT PULLEY_RATIO
    
    if displacement <= 0
        F = 0;
        return;
    end
    
    disp_in = displacement * 39.3701;  % m to inches
    force_lbf = PRELOAD + SPRING_CONSTANT * disp_in;
    F = force_lbf * 4.44822 * PULLEY_RATIO;  % Convert to N and apply pulley ratio
end

function theta3 = calculate_theta3(l2, l3, e, theta2)
    % Calculate theta3 from slider-crank geometry
    % Using: l2*sin(theta2) = e + l3*sin(-theta3)
    
    l2_sin_theta2 = l2 * sin(theta2);
    
    % Check if solution exists
    arg = l3^2 - (e - l2_sin_theta2)^2;
    if arg < 0
        theta3 = NaN;
        return;
    end
    
    % From your equation: l2*sin(theta2) = e - l3*sin(theta3)
    theta3 = -asin((e - l2_sin_theta2) / l3);
end

function displacement = calculate_slider_position(l2, l3, e, theta2)
    % Calculate slider displacement from extended dead center
    % displacement = 0 at theta2 = 0 (extended, links colinear)
    % displacement = 40mm at theta2 = pi (contracted, links overlapping)
    
    theta3 = calculate_theta3(l2, l3, e, theta2);
    if isnan(theta3)
        displacement = NaN;
        return;
    end
    
    % Current position using your equation
    l2_sin_theta2 = l2 * sin(theta2);
    x_current = l2 * cos(theta2) + sqrt(l3^2 - (e - l2_sin_theta2)^2);
    
    % Extended dead center position (theta2 = 0, displacement = 0)
    x_extended = l2 + sqrt(l3^2 - e^2);
    
    % Displacement measured from extended position
    % Positive displacement = spring being compressed (slider moving left)
    displacement = x_extended - x_current;
end

function T = calculate_torque(l2, l3, e, theta2, displacement)
    % Calculate required torque using your equation:
    % T = -F*l2*(sin(theta2) + (cos(theta2)*sin(theta3))/cos(theta3))
    
    theta3 = calculate_theta3(l2, l3, e, theta2);
    if isnan(theta3)
        T = NaN;
        return;
    end
    
    F = spring_force(displacement);
    
    % Your torque equation
    cos_theta3 = cos(theta3);
    
    if abs(cos_theta3) < 1e-6  % Avoid division by zero (toggle position)
        T = NaN;
        return;
    end
    
    T = -F * l2 * (sin(theta2) + (cos(theta2) * sin(theta3)) / cos_theta3);
end

function [theta2_range, torques, displacements, forces] = simulate_mechanism(l2, l3, e, n_points)
    % Simulate full rotation and calculate torques
    global STROKE
    
    theta2_range = linspace(0, 2*pi, n_points);
    torques = zeros(n_points, 1);
    displacements = zeros(n_points, 1);
    forces = zeros(n_points, 1);
    
    for i = 1:n_points
        theta2 = theta2_range(i);
        
        displacement = calculate_slider_position(l2, l3, e, theta2);
        if isnan(displacement)
            error('Invalid mechanism geometry at theta2 = %.2f deg', rad2deg(theta2));
        end
        
        torque = calculate_torque(l2, l3, e, theta2, displacement);
        if isnan(torque)
            error('Invalid torque calculation at theta2 = %.2f deg', rad2deg(theta2));
        end
        
        torques(i) = abs(torque);
        displacements(i) = displacement;
        forces(i) = spring_force(displacement);
    end
end

function f = objective_function(x)
    % Objective: minimize peak torque
    global STROKE
    
    l2 = x(1);
    l3 = x(2);
    e = x(3);
    
    % Quick feasibility checks
    if e <= l2 || l3^2 - e^2 <= 0
        f = 1e10;
        return;
    end
    
    % Use geometric stroke calculation
    x_extended = l2 + sqrt(l3^2 - e^2);
    x_contracted = -l2 + sqrt(l3^2 - e^2);
    stroke_calc = x_extended - x_contracted;  % This equals 2*l2
    
    if abs(stroke_calc - STROKE) > 1e-5
        f = 1e10 + abs(stroke_calc - STROKE) * 1e6;
        return;
    end
    
    try
        [~, torques, ~, ~] = simulate_mechanism(l2, l3, e, 180);
        
        % Objective: minimize peak torque with small penalty for size
        peak_torque = max(torques);
        size_penalty = 0.01 * (l2 + l3);
        
        f = peak_torque + size_penalty;
    catch
        f = 1e10;
    end
end

function [c, ceq] = nonlinear_constraints(x)
    % Nonlinear constraints
    global STROKE
    
    l2 = x(1);
    l3 = x(2);
    e = x(3);
    
    % Inequality constraints (c <= 0)
    c = [
        l2 - e + 0.001;           % e > l2 (with small margin)
        -l3^2 + e^2 + 1e-6        % l3^2 > e^2
    ];
    
    % Equality constraint (ceq = 0)
    % Use geometric stroke: stroke = 2*l2
    stroke_calc = 2*l2;
    ceq = stroke_calc - STROKE;  % Stroke must equal 40mm
end

function plot_results(l2, l3, e, theta2_range, torques, displacements, forces)
    % Plot mechanism performance
    global STROKE
    
    figure('Position', [100, 100, 1200, 900]);
    
    % Torque vs crank angle
    subplot(2, 3, 1);
    plot(rad2deg(theta2_range), torques, 'b-', 'LineWidth', 2);
    hold on;
    yline(max(torques), 'r--', 'LineWidth', 1.5);
    xlabel('Crank Angle (degrees)', 'FontSize', 11);
    ylabel('Torque (Nm)', 'FontSize', 11);
    title('Required Torque vs Crank Angle', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    legend('Torque', sprintf('Peak: %.4f Nm', max(torques)), 'Location', 'best');
    
    % Displacement vs crank angle
    subplot(2, 3, 2);
    plot(rad2deg(theta2_range), displacements*1000, 'g-', 'LineWidth', 2);
    hold on;
    yline(STROKE*1000, 'r--', 'LineWidth', 1.5);
    yline(0, 'k-', 'LineWidth', 1);
    xlabel('Crank Angle (degrees)', 'FontSize', 11);
    ylabel('Displacement (mm)', 'FontSize', 11);
    title('Slider Displacement vs Crank Angle', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    legend('Displacement', sprintf('Target: %.1f mm', STROKE*1000), 'Zero', 'Location', 'best');
    
    % Force vs crank angle
    subplot(2, 3, 3);
    plot(rad2deg(theta2_range), forces/4.44822, 'm-', 'LineWidth', 2);
    xlabel('Crank Angle (degrees)', 'FontSize', 11);
    ylabel('Force at Slider (lbf)', 'FontSize', 11);
    title('Spring Force vs Crank Angle', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    % Torque vs displacement
    subplot(2, 3, 4);
    plot(displacements*1000, torques, 'r-', 'LineWidth', 2);
    xlabel('Displacement (mm)', 'FontSize', 11);
    ylabel('Torque (Nm)', 'FontSize', 11);
    title('Torque vs Displacement', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    % Mechanical advantage
    subplot(2, 3, 5);
    effective_force = torques / l2;
    valid_idx = forces > 0.1;
    MA = zeros(size(forces));
    MA(valid_idx) = forces(valid_idx) ./ effective_force(valid_idx);
    plot(rad2deg(theta2_range), MA, 'c-', 'LineWidth', 2);
    xlabel('Crank Angle (degrees)', 'FontSize', 11);
    ylabel('Mechanical Advantage', 'FontSize', 11);
    title('Mechanical Advantage vs Crank Angle', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    % Mechanism geometry
    subplot(2, 3, 6);
    draw_mechanism_positions(l2, l3, e);
    axis equal;
    xlabel('X Position (mm)', 'FontSize', 11);
    ylabel('Y Position (mm)', 'FontSize', 11);
    title('Mechanism Geometry (8 Positions)', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    sgtitle(sprintf('Slider-Crank Analysis: l2=%.1fmm, l3=%.1fmm, e=%.1fmm', ...
        l2*1000, l3*1000, e*1000), 'FontSize', 14, 'FontWeight', 'bold');
end

function draw_mechanism_positions(l2, l3, e)
    % Draw mechanism at multiple positions
    angles = linspace(0, 2*pi, 9);
    colors = jet(8);
    
    hold on;
    
    for i = 1:8
        theta2 = angles(i);
        alpha_val = 0.3 + 0.7 * (i / 8);
        
        % Crank position
        x_crank = l2 * cos(theta2);
        y_crank = l2 * sin(theta2);
        
        % Slider position using your equation
        l2_sin_theta2 = l2 * sin(theta2);
        x_slider = l2 * cos(theta2) + sqrt(l3^2 - (e - l2_sin_theta2)^2);
        y_slider = e;
        
        % Draw links
        plot([0, x_crank]*1000, [0, y_crank]*1000, '-', 'Color', [colors(i,:), alpha_val], 'LineWidth', 2);
        plot([x_crank, x_slider]*1000, [y_crank, y_slider]*1000, '-', 'Color', [colors(i,:), alpha_val], 'LineWidth', 2);
        plot(x_slider*1000, y_slider*1000, 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 6);
        plot(x_crank*1000, y_crank*1000, 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 6);
    end
    
    % Draw ground and slider path
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
    x_min_plot = (-l2 + sqrt(l3^2 - e^2) - 0.010) * 1000;
    x_max_plot = (l2 + sqrt(l3^2 - e^2) + 0.010) * 1000;
    plot([x_min_plot, x_max_plot], [e, e]*1000, 'k--', 'LineWidth', 1);
    
    hold off;
end

function plot_theta3_analysis(l2, l3, e)
    % Create detailed theta3 analysis
    
    theta2_range = linspace(0, 2*pi, 360);
    theta3_range = zeros(size(theta2_range));
    
    for i = 1:length(theta2_range)
        theta3_range(i) = calculate_theta3(l2, l3, e, theta2_range(i));
    end
    
    figure('Position', [150, 150, 1000, 600]);
    
    % Theta3 vs Theta2
    subplot(1, 2, 1);
    plot(rad2deg(theta2_range), rad2deg(theta3_range), 'b-', 'LineWidth', 2.5);
    hold on;
    yline(0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Zero');
    yline(90, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Toggle (+90°)');
    yline(-90, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Toggle (-90°)');
    
    % Mark key positions
    plot(0, rad2deg(calculate_theta3(l2, l3, e, 0)), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '\theta_2=0°');
    plot(180, rad2deg(calculate_theta3(l2, l3, e, pi)), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '\theta_2=180°');
    
    xlabel('Crank Angle \theta_2 (degrees)', 'FontSize', 13);
    ylabel('Connecting Rod Angle \theta_3 (degrees)', 'FontSize', 13);
    title('\theta_3 vs \theta_2 Relationship', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    legend('Location', 'best');
    xlim([0 360]);
    
    % Transmission Angle
    subplot(1, 2, 2);
    transmission_angle = 90 - abs(rad2deg(theta3_range));
    plot(rad2deg(theta2_range), transmission_angle, 'Color', [0.8 0.4 0], 'LineWidth', 2.5);
    hold on;
    yline(40, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Poor (<40°)');
    yline(50, 'y--', 'LineWidth', 1.5, 'DisplayName', 'Fair (50°)');
    
    % Mark key positions
    trans_0 = 90 - abs(rad2deg(calculate_theta3(l2, l3, e, 0)));
    trans_180 = 90 - abs(rad2deg(calculate_theta3(l2, l3, e, pi)));
    plot(0, trans_0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(180, trans_180, 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    
    xlabel('Crank Angle \theta_2 (degrees)', 'FontSize', 13);
    ylabel('Transmission Angle (degrees)', 'FontSize', 13);
    title('Transmission Angle (Quality Indicator)', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    legend('Location', 'best');
    xlim([0 360]);
    ylim([0 90]);
    
    sgtitle(sprintf('Angular Analysis: l2=%.1fmm, l3=%.1fmm, e=%.1fmm', ...
        l2*1000, l3*1000, e*1000), 'FontSize', 15, 'FontWeight', 'bold');
    
    % Print statistics
    fprintf('\n--- Theta3 Analysis ---\n');
    fprintf('Theta3 at theta2=0°:    %.2f degrees\n', rad2deg(calculate_theta3(l2, l3, e, 0)));
    fprintf('Theta3 at theta2=90°:   %.2f degrees\n', rad2deg(calculate_theta3(l2, l3, e, pi/2)));
    fprintf('Theta3 at theta2=180°:  %.2f degrees\n', rad2deg(calculate_theta3(l2, l3, e, pi)));
    fprintf('Theta3 at theta2=270°:  %.2f degrees\n', rad2deg(calculate_theta3(l2, l3, e, 3*pi/2)));
    fprintf('\nTransmission angle range: %.2f° to %.2f°\n', min(transmission_angle), max(transmission_angle));
    fprintf('Worst transmission angle: %.2f° at theta2=%.1f°\n', ...
        min(transmission_angle), rad2deg(theta2_range(transmission_angle == min(transmission_angle))));
end