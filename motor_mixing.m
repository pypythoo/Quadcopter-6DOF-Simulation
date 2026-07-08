function [omega, U] = motor_mixing(T_cmd, tau_phi_cmd, tau_theta_cmd, tau_psi_cmd, p)
% MOTOR_MIXING - Converts desired thrust + torques into 4 motor speeds
L = p.L;
kT = p.k_thrust;
kD = p.k_drag;

% Mixing matrix: maps [T; tau_phi; tau_theta; tau_psi] -> [w1^2 w2^2 w3^2 w4^2]
% derived from force/torque balance for an X-frame quad
A = [ kT,      kT,      kT,      kT;  %All four motors together produce the total upward thrust.
    -kT*L,   -kT*L,    kT*L,    kT*L; %the four motors create roll.
    kT*L,   -kT*L,   -kT*L,    kT*L;  %the front motors spin faster than the back motors the drone pitches.
    -kD,      kD,     -kD,      kD  ]; %motors rotate clockwise and some anticlockwise.yaw

U_cmd = [T_cmd; tau_phi_cmd; tau_theta_cmd; tau_psi_cmd];

omega_sq = A \ U_cmd;              % solve for [w1^2 w2^2 w3^2 w4^2]
omega_sq = max(omega_sq, 0);       % motor speeds squared can't be negative

omega = sqrt(omega_sq);
omega = min(max(omega, p.omega_min), p.omega_max);  % saturate to real motor limits

% Recompute the ACTUAL thrust/torques achieved after saturation
% (important: if motors saturate, commanded ≠ actual)
omega_sq_actual = omega.^2;
U = A * omega_sq_actual;

end