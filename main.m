%% MAIN_SIM - Quadcopter 6-DOF simulation with PID attitude/altitude control
clear; clc; close all;

p = params();

% Simulation settings 
dt = 0.01;         % timestep (s)
t_end = 15;         % total sim time (s)
t_vec = 0:dt:t_end;
N = length(t_vec);

% Initial state: everything zero, drone sitting at origin 
state = zeros(12,1);

% Reference (what we want the drone to do) 
% Command: climb to 3m altitude and hold level (phi=theta=psi=0)
ref.z = 3.0;
ref.phi = 0;
ref.theta = 0;
ref.psi = 0;

% Controller memory (integral terms, previous errors) 
ctrl_state.int_z = 0;
ctrl_state.int_phi = 0; ctrl_state.int_theta = 0; ctrl_state.int_psi = 0;
ctrl_state.prev_e_phi = 0; ctrl_state.prev_e_theta = 0; ctrl_state.prev_e_psi = 0;

% Storage for plotting 
state_history = zeros(12, N);
U_history = zeros(4, N);
state_history(:,1) = state;

% Main time-stepping loop 
for k = 1:N-1
    % Controller computes desired thrust/torques from current error
    [U_cmd, ctrl_state] = pid_controller(state, ref, ctrl_state, dt, p);

    %  Motor mixing converts commands to motor speeds (with saturation), then back to the ACTUAL achievable thrust/torques
    [omega, U_actual] = motor_mixing(U_cmd(1), U_cmd(2), U_cmd(3), U_cmd(4), p);

    %  Integrate the dynamics forward by one timestep using the actual U
    [~, y] = ode45(@(t,s) quad_dynamics(t, s, U_actual, p), [t_vec(k) t_vec(k+1)], state);
    state = y(end,:)';

    %  results
    state_history(:,k+1) = state;
    U_history(:,k) = U_actual;
end

save('sim_results.mat', 't_vec', 'state_history', 'U_history', 'p');

disp('Simulation complete. Results saved to sim_results.mat');
plot_results(t_vec, state_history, U_history);
animate_quad(t_vec, state_history, p);