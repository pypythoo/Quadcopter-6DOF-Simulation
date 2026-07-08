function p = params()
% PARAMS - Physical properties of the quadcopter

p.m = 1.0;              % mass (kg)
p.g = 9.81;             % gravity (m/s^2)
p.L = 0.25;             % arm length, center to motor (m)

% Moments of inertia (kg*m^2) - treating the drone as a symmetric cross frame
p.Ixx = 0.0142;
p.Iyy = 0.0142;
p.Izz = 0.0284;

% Motor/rotor properties
p.k_thrust = 2.98e-6;   % thrust coefficient (N per (rad/s)^2)
p.k_drag   = 1.14e-7;   % drag/torque coefficient (N*m per (rad/s)^2)

% Motor speed limits (rad/s)
p.omega_min = 0;
p.omega_max = 900;

% Hover motor speed (all 4 motors, for trim/reference)
p.omega_hover = sqrt((p.m * p.g) / (4 * p.k_thrust));

end