function [U_cmd, ctrl_state] = pid_controller(state, ref, ctrl_state, dt, p)
% PID_CONTROLLER - Computes [T; tau_phi; tau_theta; tau_psi] commands

% state = current 12x1 state (see quad_dynamics.m)
% ref   = struct with desired z, phi, theta, psi
% ctrl_state = struct carrying integral & previous-error terms between calls
% dt    = timestep (s)

%  Extract current values 
z     = state(3);
w     = state(6);
phi   = state(7);
theta = state(8);
psi   = state(9);

%  Errors 
e_z     = ref.z - z;
e_phi   = ref.phi - phi;
e_theta = ref.theta - theta;
e_psi   = wrapToPi(ref.psi - psi);

% --- Gains (tune these to change response speed/damping) ---
Kp_z = 12;   Ki_z = 4;   Kd_z = 8;
Kp_att = 6;  Ki_att = 0.3; Kd_att = 1.8;

%  Altitude PID (outputs thrust ABOVE hover baseline) 
ctrl_state.int_z = ctrl_state.int_z + e_z * dt;
d_z = -w;   % derivative of altitude error ≈ -vertical velocity
T_cmd = p.m * p.g + Kp_z*e_z + Ki_z*ctrl_state.int_z + Kd_z*d_z;

%  Attitude PID (roll, pitch, yaw) 
ctrl_state.int_phi   = ctrl_state.int_phi   + e_phi   * dt;
ctrl_state.int_theta = ctrl_state.int_theta + e_theta * dt;
ctrl_state.int_psi   = ctrl_state.int_psi   + e_psi   * dt;

d_phi   = (e_phi   - ctrl_state.prev_e_phi)   / dt;
d_theta = (e_theta - ctrl_state.prev_e_theta) / dt;
d_psi   = (e_psi   - ctrl_state.prev_e_psi)   / dt;

tau_phi_cmd   = Kp_att*e_phi   + Ki_att*ctrl_state.int_phi   + Kd_att*d_phi;
tau_theta_cmd = Kp_att*e_theta + Ki_att*ctrl_state.int_theta + Kd_att*d_theta;
tau_psi_cmd   = Kp_att*e_psi   + Ki_att*ctrl_state.int_psi   + Kd_att*d_psi;  
% Save error history for next derivative calc 
ctrl_state.prev_e_phi   = e_phi;
ctrl_state.prev_e_theta = e_theta;
ctrl_state.prev_e_psi   = e_psi;

U_cmd = [T_cmd; tau_phi_cmd; tau_theta_cmd; tau_psi_cmd];

end