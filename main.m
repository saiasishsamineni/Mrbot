clear
clc
clear Traj_con

% Main file
% Run a simulation for a semi-circular blood vessel path

% Determine MRbot parameters
Rsphere = 2 * 10^-6; %--> Sphere radius (m)
Density = 7860; %--> Iron density (kg/m3)
Mmag = 1.5; %--> Magnetization constant (Tesla)

% Determine path segmentation parameters
Np = 1000; %--> Number of segments
R_path = 0.1; %--> Path/semi-circle radius
Rgc = 10 * 10^-6 * ones(1, Np); 
  %--> Radius of guidance corridor in every segment (assume constant for all segments)

% Execute path planning (position and velocity profiles)
[P_prof, V_prof] = PLM_semi_circle(Np, R_path, Rsphere, Rgc);

% Construct MRbot object (call it whatever, I called it Robot_1)
Robot_1 = MRbot(Rsphere, Mmag, Density, P_prof(:,1).');

% Simulation loop
sim_time = 16.35; %--> Simulation time (sec.)
dt = 0.01; %--> Simulation sampling time (sec.)
Pos_hist = zeros(4, round(sim_time/dt) + 1); %--> History of position vectors
Vel_hist = zeros(4, round(sim_time/dt) + 1); %--> History of velocity vectors
Pos_hist(1:3, 1) = P_prof(:,1); 

for i = 1 : round(sim_time/dt)
    % Trajectory control action
    % Pick the closest point in path to current point
    [min_dist, I_min_dist] = min(sqrt(sum((Pos_hist(1:3, i) - P_prof).^2)));
    
    Grad = Traj_con(Vel_hist(1:3, i).', V_prof(:, I_min_dist).', Pos_hist(1:3, i).', ...
        P_prof(:, I_min_dist).', dt, Rsphere, Mmag); %--> Magnetic field gradient

    % Model update
    Robot_1 = solve(Robot_1, Grad, dt); %--> Model solve

    % Record history
    Pos_hist(:, i + 1) = [getpos(Robot_1).'; i * dt];
    Vel_hist(:, i + 1) = [getvel(Robot_1).'; i * dt];
end

% Plot results
% Path plot
figure
plot3(P_prof(1, :), P_prof(2, :), P_prof(3, :), 'r', 'LineWidth', 2)
hold on 
plot3(Pos_hist(1, :), Pos_hist(2, :), Pos_hist(3, :), '--g', 'LineWidth', 2)
grid on
xlabel('X axis (m)','FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
ylabel('Y axis (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
zlabel('Z axis (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
set (gca, 'fontweight', 'bold', 'FontSize', 18) 
set(gcf,'units','points','position',[.0, .0, 330, 330])
legend('Desired path','Actual path', 'Location', 'Best')

% Trajectory plot
figure
dP_prof = diff(P_prof.').';
dPos_hist = diff(Pos_hist.').';
quiver3(P_prof(1, 1 : 100 : end - 1), P_prof(2, 1 : 100 : end - 1), P_prof(3, 1 : 100 : end - 1), ...
    dP_prof(1, 1 : 100 : end), dP_prof(2, 1 : 100 : end), dP_prof(3, 1 : 100 : end), ...
    'r')
hold on
quiver3(Pos_hist(1, 1 : 100 : end - 1), Pos_hist(2, 1 : 100 : end - 1), Pos_hist(3, 1 : 100 : end - 1), ...
    dPos_hist(1, 1 : 100 : end), dPos_hist(2, 1 : 100 : end), dPos_hist(3, 1 : 100 : end), ...
    'g')
xlabel('X axis (m)','FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
ylabel('Y axis (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
zlabel('Z axis (m)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k')
set (gca, 'fontweight', 'bold', 'FontSize', 18) 
set(gcf,'units','points','position',[.0, .0, 330, 330])
legend('Desired traj.','Actual traj.', 'Location', 'Best')