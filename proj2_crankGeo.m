
clear; clc; close all;

Sy = 2.76e+8; % N/m^2, Aluminum 6061-T6
T_m = 1.5; % Nm, from the gearbox output
FOS = 2;
S_allow = Sy/FOS;

% geometry - starting guesses
l = 20/1000; % m
t = 4/1000; % m
h = 6/1000; % m 

h_array = linspace(2/1000, 40/1000, 100);

S_bending_h= 6*T_m./(t .* (h_array).^2);

figure(1)
plot(h_array*1000, S_bending_h);
hold on;
yline(S_allow);
legend("Bending stress", "Yield strength, FOS = 2");
xlabel("Height (mm)");
ylabel("Bending stress (Pa)");

h_optimal = 5/1000; % m, picked from the plot

t_array = linspace(3/1000, 13/1000, 100);

S_bending_t= 6*T_m./(t_array .* (h_optimal).^2);

figure(2)
plot(t_array*1000, S_bending_t);
hold on;
yline(S_allow);
legend("Bending stress", "Yield strength, FOS = 2");
xlabel("Thickness (mm)");
ylabel("Bending stress (MPa)");
