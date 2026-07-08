function plot_results(t_vec, state_history, U_history)
% PLOT_RESULTS - Plots altitude, attitude, and control effort over time

z     = state_history(3,:);
phi   = rad2deg(state_history(7,:));
theta = rad2deg(state_history(8,:));
psi   = rad2deg(state_history(9,:));

figure('Name','Quadcopter Simulation Results','NumberTitle','off');

subplot(3,1,1);
plot(t_vec, z, 'b', 'LineWidth', 1.5);
yline(3, '--r', 'Target altitude');
xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Response'); grid on;

subplot(3,1,2);
plot(t_vec, phi, 'r', t_vec, theta, 'g', t_vec, psi, 'b', 'LineWidth', 1.2);
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude Response'); grid on;

subplot(3,1,3);
plot(t_vec(1:end-1), U_history(1,1:end-1), 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Total Thrust Command'); grid on;

sgtitle('Quadcopter PID Control Simulation');
saveas(gcf, 'sim_plots.png');

end