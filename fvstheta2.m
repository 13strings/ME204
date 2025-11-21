clear all; close all; clc;


preload = 24.91 ; % Newtons, 5.6 lbf
k = 2084 ; % N/m, 11.9lbF
theta_crank = linspace(0,360,361); % deg, = theta2
r_crank = 0.020; % m
k_motor = 0.0021; 

t_friction  = 0.000709; % N-m
gear_ratio = 1066.7;

%% solving for torque 
moment_arm = r_crank*sind(theta_crank);

displacement = r_crank - r_crank*cosd(theta_crank);

F = preload + k.*displacement;

t_gearbox = moment_arm .* F;

figure(1)
plot(theta_crank, t_gearbox);
title("Moment over revolution");
xlabel("Crank angle (deg)");
ylabel("Moment (N-m)");

% avg_m = mean(abs(t_gearbox));
% max_m = max(abs(t_gearbox));

%% solving for current from motor over cycle

current = (t_gearbox/gear_ratio + t_friction)./k_motor;
current_pos = max(current, 0);

figure(2)
plot(theta_crank, current_pos);
title("Current over revolution");
xlabel("Crank angle (deg)");
ylabel("Current (A)");

%% solving for speed 
V_app = 2.5; % V
R = 0.8; % Ohm

speed_motor = (V_app - current_pos*R) / k_motor; % rad /s
speed_crank = speed_motor / gear_ratio;

figure(3)
plot(theta_crank, speed_motor * 180/pi / 60);
title("Motor Speed over 1 crank revolution");
xlabel("Crank Angle (deg)");
ylabel("Motor Speed (RPM)");


figure(4)
plot(theta_crank, speed_crank * 180/ pi / 60);
title("Crank Speed over 1 crank revolution");
xlabel("Crank Angle (deg)");
ylabel("Crank Speed (RPM)");


%% integrating for current / time trapz stuff T__T

theta_crank_rad = theta_crank * pi / 180;
dtheta_crank = diff(theta_crank_rad);
speed_crank_avg = (speed_crank(1:end-1) + speed_crank(2:end)) / 2;
dt = dtheta_crank ./ speed_crank_avg;
time = [0, cumsum(dt)];

T_cycle = time(end);
average_current = trapz(time, current_pos) / T_cycle

%% plot of pos over time to 

theta_motor = theta_crank * gear_ratio;

figure(5)
plot(time, theta_crank);
xlabel('Time (s)');
ylabel('Crank Angle (deg)');
title('Crank Angle over Time');

figure(6)
plot(time, theta_motor);
xlabel('Time (s)');
ylabel("Motor theta  (deg)");
title("Motor theta over time");


