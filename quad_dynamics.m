function dstate = quad_dynamics(t, state, U, p)
% QUAD_DYNAMICS - 6-DOF nonlinear equations of motion for a quadcopter

phi   = state(7);  %roll angle (rotation about the x-axis).
theta = state(8);  %pitch angle (rotation about the y-axis).(q)
psi   = state(9);  %yaw angle (rotation about the z-axis).(r)
p_rate = state(10);
q_rate = state(11);
r_rate = state(12);

T        = U(1);
tau_phi  = U(2);
tau_theta= U(3);
tau_psi  = U(4);

% Rotation matrix (body -> world), ZYX Euler convention 
R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

% Translational dynamics
% Thrust acts along body z-axis; gravity acts along world -z
thrust_body = [0; 0; T];
accel_world = (R * thrust_body) / p.m - [0; 0; p.g]; %newtons second law 

% Rotational dynamics (Euler's equations) 
p_dot = (tau_phi   - (p.Izz - p.Iyy) * q_rate * r_rate) / p.Ixx;
q_dot = (tau_theta - (p.Ixx - p.Izz) * p_rate * r_rate) / p.Iyy;
r_dot = (tau_psi   - (p.Iyy - p.Ixx) * p_rate * q_rate) / p.Izz;

% --- Euler angle rates from body angular rates ---
phi_dot   = p_rate + sin(phi)*tan(theta)*q_rate + cos(phi)*tan(theta)*r_rate;
theta_dot = cos(phi)*q_rate - sin(phi)*r_rate;
psi_dot   = sin(phi)/cos(theta)*q_rate + cos(phi)/cos(theta)*r_rate;

% --- Assemble derivative ---
dstate = zeros(12,1);
dstate(1:3)  = state(4:6);       % dx/dt = velocity
dstate(4:6)  = accel_world;      % dv/dt = acceleration
dstate(7)    = phi_dot;
dstate(8)    = theta_dot;
dstate(9)    = psi_dot;
dstate(10)   = p_dot;
dstate(11)   = q_dot;
dstate(12)   = r_dot;

end