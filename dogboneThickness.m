clc; clear all ; close all;
% Your specific case
d_bore = 10;           % mm
thickness = 4.7;       % mm
force = 110;           % N
yield_strength = 200;  % MPa (mild steel or aluminum)

[D_min, results] = analyze_link_bore(d_bore, thickness, force, yield_strength);

% Display table
fprintf('\n D(mm) | d/D  |  Kt  | σmax(MPa) | FOS\n');
fprintf('-------|------|------|-----------|-----\n');
for i = 1:5:size(results,1)
    fprintf(' %5.1f | %.2f | %.2f |   %6.2f  | %5.1f\n', ...
        results(i,1), results(i,2), results(i,3), results(i,5), results(i,6));
end

function [D_outer, results] = analyze_link_bore(d_bore, thickness, force, material_yield)
    % Inputs:
    %   d_bore: bore diameter (mm)
    %   thickness: link thickness (mm)
    %   force: tensile force (N)
    %   material_yield: yield strength (MPa)
    
    % Test range of outer diameters
    D_range = (d_bore + 2):0.5:30;  % Start at d+2mm minimum
    
    n = length(D_range);
    results = zeros(n, 6);
    
    for i = 1:n
        D = D_range(i);
        ratio = d_bore / D;
        
        % Calculate Kt using Pilkey's formula
        Kt = 3.0 - 3.14*ratio + 3.667*ratio^2 - 1.527*ratio^3;
        
        % Net area
        A_net = (D - d_bore) * thickness;
        
        % Nominal stress
        sigma_nom = force / A_net;
        
        % Maximum stress
        sigma_max = Kt * sigma_nom;
        
        % Factor of safety
        FOS = material_yield / sigma_max;
        
        % Store results
        results(i,:) = [D, ratio, Kt, A_net, sigma_max, FOS];
    end
    
    % Find minimum diameter for FOS >= 3
    idx = find(results(:,6) >= 2, 1, 'first');
    D_outer = results(idx, 1);
    
    % Display results
    fprintf('Minimum outer diameter for FOS >= 2: %.1f mm\n', D_outer);
    
    % Plot
    figure;
    subplot(2,1,1);
    plot(results(:,1), results(:,5), 'LineWidth', 2);
    xlabel('Outer Diameter (mm)');
    ylabel('Max Stress (MPa)');
    title('Maximum Stress vs Outer Diameter');
    grid on;
    
    subplot(2,1,2);
    plot(results(:,1), results(:,6), 'LineWidth', 2);
    hold on;
    yline(2, 'r--', 'FOS = 2');
    xlabel('Outer Diameter (mm)');
    ylabel('Factor of Safety');
    title('Factor of Safety vs Outer Diameter');
    grid on;
end