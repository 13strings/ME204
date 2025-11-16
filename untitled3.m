clear; clc; close all;

%% Material Properties
Sy = 2.76e+8;  % Pa (276 MPa), Aluminum 6061-T6
FOS = 2;
S_allow = Sy / FOS;  % Pa (138 MPa)

%% Loading
M_b = 2.22;  % Nm, BENDING MOMENT (not torque!)

fprintf('================================================================\n');
fprintf('CRANK LINK SIZING\n');
fprintf('================================================================\n');
fprintf('Material: Aluminum 6061-T6\n');
fprintf('Yield Strength: %.0f MPa\n', Sy/1e6);
fprintf('Factor of Safety: %.1f\n', FOS);
fprintf('Allowable Stress: %.0f MPa\n', S_allow/1e6);
fprintf('Bending Moment: %.2f Nm\n', M_b);
fprintf('================================================================\n\n');

%% Analysis 1: Vary Height (h) with Fixed Thickness
t = 6/1000;  % m, fixed thickness
h_array = linspace(10/1000, 40/1000, 100);

% Bending stress: σ = M*c/I = 6*M/(t*h²)
S_bending_h = 6 * M_b ./ (t * h_array.^2);  % Pa

figure(1)
plot(h_array*1000, S_bending_h/1e6, 'b-', 'LineWidth', 2);  % Convert to MPa
hold on;
yline(S_allow/1e6, 'r--', 'LineWidth', 2);  % Convert to MPa
legend('Bending stress', sprintf('Allowable = %.0f MPa (Sy/FOS)', S_allow/1e6), ...
       'FontSize', 11, 'Location', 'best');
xlabel('Height h (mm)', 'FontSize', 12);
ylabel('Bending Stress (MPa)', 'FontSize', 12);
title(sprintf('Bending Stress vs Height (t = %.1f mm)', t*1000), ...
      'FontSize', 13, 'FontWeight', 'bold');
grid on;

% Find minimum safe height
idx_safe = find(S_bending_h <= S_allow, 1, 'first');
if ~isempty(idx_safe)
    h_min = h_array(idx_safe);
    fprintf('Minimum safe height (t=%.1f mm): h = %.2f mm\n', t*1000, h_min*1000);
    fprintf('  Stress at this point: %.1f MPa\n\n', S_bending_h(idx_safe)/1e6);
    
    % Add vertical line on plot
    xline(h_min*1000, 'g--', 'LineWidth', 1.5, 'Label', sprintf('h_{min}=%.1fmm', h_min*1000));
else
    fprintf('WARNING: No safe height found! All stresses exceed allowable.\n\n');
    h_min = max(h_array);
end

h_optimal = h_min;  % Use calculated minimum instead of picking from plot

%% Analysis 2: Vary Thickness (t) with Optimal Height
t_array = linspace(3/1000, 13/1000, 100);

% Bending stress: σ = 6*M/(t*h²)
S_bending_t = 6 * M_b ./ (t_array * h_optimal^2);  % Pa

figure(2)
plot(t_array*1000, S_bending_t/1e6, 'b-', 'LineWidth', 2);  % Convert to MPa
hold on;
yline(S_allow/1e6, 'r--', 'LineWidth', 2);  % Convert to MPa
legend('Bending stress', sprintf('Allowable = %.0f MPa (Sy/FOS)', S_allow/1e6), ...
       'FontSize', 11, 'Location', 'best');
xlabel('Thickness t (mm)', 'FontSize', 12);
ylabel('Bending Stress (MPa)', 'FontSize', 12);
title(sprintf('Bending Stress vs Thickness (h = %.1f mm)', h_optimal*1000), ...
      'FontSize', 13, 'FontWeight', 'bold');
grid on;

% Find minimum safe thickness
idx_safe_t = find(S_bending_t <= S_allow, 1, 'first');
if ~isempty(idx_safe_t)
    t_min = t_array(idx_safe_t);
    fprintf('Minimum safe thickness (h=%.1f mm): t = %.2f mm\n', h_optimal*1000, t_min*1000);
    fprintf('  Stress at this point: %.1f MPa\n\n', S_bending_t(idx_safe_t)/1e6);
    
    % Add vertical line on plot
    xline(t_min*1000, 'g--', 'LineWidth', 1.5, 'Label', sprintf('t_{min}=%.1fmm', t_min*1000));
else
    fprintf('WARNING: No safe thickness found!\n\n');
    t_min = max(t_array);
end

%% Final Design Summary
fprintf('================================================================\n');
fprintf('RECOMMENDED DESIGN\n');
fprintf('================================================================\n');
fprintf('Minimum Dimensions:\n');
fprintf('  Height (h):     %.2f mm\n', h_optimal*1000);
fprintf('  Thickness (t):  %.2f mm\n', t_min*1000);
fprintf('  Cross-section:  %.2f × %.2f mm\n', t_min*1000, h_optimal*1000);
fprintf('  Area:           %.1f mm²\n\n', t_min*h_optimal*1e6);

% Calculate actual stress with final dimensions
S_final = 6 * M_b / (t_min * h_optimal^2);
FOS_actual = Sy / S_final;

fprintf('Final Performance:\n');
fprintf('  Bending stress: %.1f MPa\n', S_final/1e6);
fprintf('  Allowable:      %.1f MPa\n', S_allow/1e6);
fprintf('  Utilization:    %.1f%%\n', S_final/S_allow*100);
fprintf('  Actual FOS:     %.2f\n', FOS_actual);
fprintf('================================================================\n');