%% Offset Slider-Crank Optimization Script
% Optimizes l2, l3, and e for minimum torque while meeting stroke requirement
% Handles both e < l2 and e > l2 cases

clear; clc; close all;

%% Constants
global STROKE PRELOAD SPRING_CONSTANT PULLEY_RATIO K_SPRING F_PRELOAD

STROKE = 40e-3;          % 40mm in meters
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

%% Run Both Optimizations

fprintf('CASE 1: Optimizing with e < l2 constraint...\n');
fprintf('-------------------------------------------------------------\n');
[l2_opt1, l3_opt1, e_opt1, fval1] = optimize_case(1);

fprintf('\n\nCASE 2: Optimizing with e > l2 constraint...\n');
fprintf('-------------------------------------------------------------\n');
[l2_opt2, l3_opt2, e_opt2, fval2] = optimize_case(2);

fprintf('\n\n=============================================================\n');
fprintf('COMPARISON OF BOTH CASES\n');
fprintf('=============================================================\n');
fprintf('CASE 1 (e < l2):\n');
fprintf('  l2 = %.2f mm, l3 = %.2f mm, e = %.2f mm\n', l2_opt1*1000, l3_opt1*1000, e_opt1*1000);
fprintf('  Peak Torque = %.4f Nm (%.2f oz-in)\n', fval1, fval1*141.612);

fprintf('\nCASE 2 (e > l2):\n');
fprintf('  l2 = %.2f mm, l3 = %.2f mm, e = %.2f mm\n', l2_opt2*1000, l3_opt2*1000, e_opt2*1000);
fprintf('  Peak Torque = %.4f Nm (%.2f oz-in)\n', fval2, fval2*141.612);

if fval1 < fval2
    fprintf('\n*** CASE 1 (e < l2) IS BETTER - %.1f%% lower peak torque ***\n', ...
        (1-fval1/fval2)*100);
    best_case = 1;
    l2_best = l2_opt1; l3_best = l3_opt1; e_best = e_opt1;
else
    fprintf('\n*** CASE 2 (e > l2) IS BETTER - %.1f%% lower peak torque ***\n', ...
        (1-fval2/fval1)*100);
    best_case = 2;
    l2_best = l2_opt2; l3_best = l3_opt2; e_best = e_opt2;
end
fprintf('=============================================================\n');

%% Plot best case
[theta2_range, torques, displacements, forces] = simulate_mechanism(l2_best, l3_best, e_best, 360);
plot_results(l2_best, l3_best, e_best, theta2_range, torques, displacements, forces, best_case);

%% ========================================================================
% OPTIMIZATION FUNCTION
%% ========================================================================

function [l2_opt, l3_opt, e_opt, fval] = optimize_case(case_num)
    % case_num: 1 for e < l2, 2 for e > l2
    
    % Bounds (in meters)
    lb = [0.010, 0.050, 0.001];  % Lower bounds [l2, l3, e]
    ub = [0.100, 0.300, 0.150];  % Upper bounds
    
    % Initial guess
    if case_num == 1
        x0 = [0.025, 0.150, 0.015];  % e < l2
    else
        x0 = [0.020, 0.150, 0.040];  % e > l2
    end
    
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
    [x_opt, fval, exitflag, output] = fmincon(@(x) objective_function(x, case_num), x0, ...
        [], [], [], [], lb, ub, @(x) nonlinear_constraints(x, case_num), options);
    
    l2_opt = x_opt(1);
    l3_opt = x_opt(2);
    e_opt = x_opt(3);
    
    % Display results for this case
    fprintf('\nOptimization Results:\n');
    fprintf('  Crank length (l2):        %.2f mm\n', l2_opt*1000);
    fprintf('  Connecting rod (l3):      %.2f mm\n', l3_opt*1000);
    fprintf('  Offset distance (e):      %.2f mm\n', e_opt*1000);
    fprintf('  Ratio e/l2:               %.3f\n', e_opt/l2_opt);
    
    if case_num == 1
        stroke_calc = 2*l2_opt;
    else
        stroke_calc = 2*sqrt(l3_opt^2 - e_opt^2) - 2*l2_opt;
    end
    fprintf('  Stroke verification:      %.4f mm\n', stroke_calc*1000);
    
    [~, torques, ~, ~] = simulate_mechanism(l2_opt, l3_opt, e_opt, 360);
    fprintf('  Peak torque:              %.4f Nm (%.2f oz-in)\n', max(torques), max(torques)*141.612);
    fprintf('  Average torque:           %.4f Nm (%.2f oz-in)\n', mean(torques), mean(torques)*141.612);
    fprintf('  RMS torque:               %.4f Nm (%.2f oz-in)\n', rms(torques), rms(torques)*141.612);
end

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
    % Calculate slider displacement from reference position
    % Using: x = l2*cos(theta2) + sqrt(l3^2 - (e - l2*sin(theta2))^2)
    
    theta3 = calculate_theta3(l2, l3, e, theta2);
    if isnan(theta3)
        displacement = NaN;
        return;
    end
    
    % Reference position at theta2 = 0 (maximum extension)
    x_ref = l2 + sqrt(l3^2 - e^2);
    
    % Current position using your equation
    l2_sin_theta2 = l2 * sin(theta2);
    x = l2 * cos(theta2) + sqrt(l3^2 - (e - l2_sin_theta2)^2);
    
    % Displacement from reference (positive = spring stretching)
    displacement = x_ref - x;
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
        
        % Clamp displacement to working range [0, 40mm]
        displacement = max(0, min(STROKE, displacement));
        
        torque = calculate_torque(l2, l3, e, theta2, displacement);
        if isnan(torque)
            error('Invalid torque calculation at theta2 = %.2f deg', rad2deg(theta2));
        end
        
        torques(i) = abs(torque);
        displacements(i) = displacement;
        forces(i) = spring_force(displacement);
    end
end

function f = objective_function(x, case_num)
    % Objective: minimize peak torque
    global STROKE
    
    l2 = x(1);
    l3 = x(2);
    e = x(3);
    
    % Quick feasibility checks
    if l3^2 - e^2 <= 0
        f = 1e10;
        return;
    end
    
    % Check case-specific constraint
    if case_num == 1 && e >= l2  % Case 1: need e < l2
        f = 1e10;
        return;
    elseif case_num == 2 && e <= l2  % Case 2: need e > l2
        f = 1e10;
        return;
    end
    
    % Stroke constraint check
    if case_num == 1
        stroke_calc = 2*l2;
    else
        stroke_calc = 2*sqrt(l3^2 - e^2) - 2*l2;
    end
    
    if abs(stroke_calc - STROKE) > 1e-5
        f = 1e10 + abs(stroke_calc - STROKE) * 1e6;
        return;
    end
    
    try
        [~, torques, ~, ~] = simulate_mechanism(l2, l3, e, 180);
        
        % Objective: minimize peak torque with small penalty for size
        peak_torque = max(torques);
        size_penalty = 0.001 * (l2 + l3);  % Small penalty for compactness
        
        f = peak_torque + size_penalty;
    catch
        f = 1e10;
    end
end

function [c, ceq] = nonlinear_constraints(x, case_num)
    % Nonlinear constraints
    global STROKE
    
    l2 = x(1);
    l3 = x(2);
    e = x(3);
    
    % Inequality constraints (c <= 0)
    c = -l3^2 + e^2 + 1e-6;  % l3^2 > e^2 (must be real)
    
    % Add case-specific constraint
    if case_num == 1
        c = [c; e - l2 + 0.0001];  % e < l2 (Case 1)
    else
        c = [c; l2 - e + 0.0001];  % e > l2 (Case 2)
    end
    
    % Equality constraint (ceq = 0)
    if case_num == 1
        % Case 1: stroke = 2*l2
        stroke_calc = 2*l2;
    else
        % Case 2: stroke = 2*sqrt(l3^2 - e^2) - 2*l2
        stroke_calc = 2*sqrt(l3^2 - e^2) - 2*l2;
    end
    
    ceq = stroke_calc - STROKE;  % Stroke must equal 40mm
end

function plot_results(l2, l3, e, theta2_range, torques, displacements, forces, case_num)
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
    xlabel('Crank Angle (degrees)', 'FontSize', 11);
    ylabel('Displacement (mm)', 'FontSize', 11);
    title('Slider Displacement vs Crank Angle', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    legend('Displacement', sprintf('Target: %.1f mm', STROKE*1000), 'Location', 'best');
    
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
    
    case_str = sprintf('Case %d (e %s l2)', case_num, char(60 + 2*(case_num-1)));
    sgtitle(sprintf('BEST DESIGN - %s: l2=%.1fmm, l3=%.1fmm, e=%.1fmm', ...
        case_str, l2*1000, l3*1000, e*1000), 'FontSize', 14, 'FontWeight', 'bold');
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
    x_min_plot = (l2 - sqrt(l3^2 - e^2) - 0.010) * 1000;
    x_max_plot = (l2 + sqrt(l3^2 - e^2) + 0.010) * 1000;
    plot([x_min_plot, x_max_plot], [e, e]*1000, 'k--', 'LineWidth', 1);
    
    hold off;
end