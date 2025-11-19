clear all; close all; clc;

preload = 24.91 ; % Newtons, 5.6 lbf
k = 2084 ; % N/m, 11.9lbF

theta2 = linspace(0,360,361); % deg

r_crank = 0.020; % m

moment_arm = r_crank*sind(theta2);

displacement = r_crank - r_crank*cosd(theta2);

F = preload + k.*displacement;

moment = moment_arm .* F;

figure(1)
plot(theta2, moment);
title("Moment over revolution");
xlabel("Theta 2 (deg)");
ylabel("Moment (N-m)");

avg_m = mean(abs(moment));
max_m = max(abs(moment));