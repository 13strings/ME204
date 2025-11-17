% sizing holes for links/bearings stress conc

clear; clc; close all;

Sy = 2.76e+8; % N/m^2, Aluminum 6061-T6
T_m = 2.22; % Nm, from the gearbox output
FOS = 2;
S_allow = Sy/FOS;

d = 4/1000; %  from thickness of crankGeo
t_array = linspace(1/1000, 5/1000, 100); % defined as r_o of rod - r_i of rod

F_spring = 110; % N

r_i = 5/1000; % also have to optimize this

r_c = r_i + t_array/2;

S_bending = F_spring/2 .* (r_c) .* (r_c - r_i) ./ (1/12 .* d .* (t_array).^3);

S_axial = F_spring/2 ./ t_array / d;

S_combined = S_bending + S_axial;

figure(1)
plot(t_array*1000,S_combined);
hold on;
yline(S_allow);
legend("Combined stress", "Yield strength, FOS = 2");
xlabel("thickness of radial circle (mm)");
ylabel("Combined stress (MPa)");

t_optimal = 2.65/1000; % from plot

r_i_array = linspace(1/1000, 10/1000, 100);

r_c_array = r_i_array + t_optimal/2;

S_bending = F_spring/2 .* (r_c_array) .* (r_c_array - r_i_array) ./ (1/12 .* d .* (t_optimal).^3);

S_axial = F_spring/2 ./ t_optimal / d;

S_combined = S_bending + S_axial;

figure(2)
plot(r_i_array*1000,S_combined);
hold on;
yline(S_allow);
legend("Combined stress", "Yield strength, FOS = 2");
xlabel("inner radius (mm)");
ylabel("Combined stress (MPa)");