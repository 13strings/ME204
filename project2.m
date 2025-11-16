% motor parameters
k = ;
t_fric = ;
R = 1; % ohm (paul got 1.6 ohm)
n_g = ;


% variables
v_app = linspace(0.5,2.5, 10);
r_g = ;
i_app = ;

% solve for torque_load and theta_motor

t_load = k * i_app - t_fric; 

theta_motor = (v_app - i_app * R) / k; % rad/s

t_gear_out = n_g * r_g * t_load;

% solve for output pull force (F_cy)


