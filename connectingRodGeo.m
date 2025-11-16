clear; clc; close all;

Sy = 2.76e+8; % N/m^2, Aluminum 6061-T6
T_m = 2.22; % Nm, from the gearbox output
FOS = 2;
S_allow = Sy/FOS;

F_spring = 110; % N

% geometry - starting guesses
l = 130/1000; % m
t = 4/1000; % m
w_array = linspace(1/1000, 12/1000, 100);



S_tension = F_spring  ./ (w_array .* t);

figure(1)
plot(w_array, S_tension);
hold on;
yline(S_allow);
legend("Tensile stress", "Yield strength, FOS = 2");
xlabel("Width (mm)");
ylabel("Tensile stress (Pa)");

