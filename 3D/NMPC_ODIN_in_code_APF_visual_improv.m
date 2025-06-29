clear all; clc;
%% Define Waypoints, Obstacle, and Parameters
wpt.pos.x = [0, 2, 4, 7, 10];
wpt.pos.y = [0, 2, 5, 4, 12];

%% Generate Smooth Path Using Spline
spline = cscvn([wpt.pos.x; wpt.pos.y]);
t_spline = linspace(spline.breaks(1), spline.breaks(end), 10000);
path_points = fnval(spline, t_spline);
path_x = path_points(1,:);
path_y = path_points(2,:);
path_z = linspace(0, 12, 10000);

% Compute first derivatives using gradient
dt = diff(t_spline(1:2)); 
dx = gradient(path_x, dt);
dy = gradient(path_y, dt);
dz = gradient(path_z, dt);
yaw = atan2(dy, dx);
dyaw = gradient(yaw,dt);

% Compute second derivatives if needed
ddx = gradient(dx, dt);
ddy = gradient(dy, dt);
ddz = gradient(dz, dt);
ddyaw = gradient(dyaw,dt);

% Construct the derivative matrix
ref_matrix = [path_x;path_y;path_z;zeros(1,10000);zeros(1,10000);yaw];
ref_matrix_derivative = [dx; dy; dz; zeros(1,10000);zeros(1,10000);dyaw];
ref_matrix_derivative_two = [ddx;ddy;ddz;zeros(1,10000);zeros(1,10000);ddyaw];
nx = 12;
ny = 6;
nu = 6;

NMPC_ODIN = nlmpc(nx,ny,nu);

Ts = 0.1;
NMPC_ODIN.Ts = Ts;
NMPC_ODIN.PredictionHorizon = 15;
NMPC_ODIN.ControlHorizon = 10;

%% Plant Model---------------------------------------------------------------
NMPC_ODIN.Model.StateFcn = "ODIN_dynamics";
NMPC_ODIN.Model.OutputFcn = @(x, u) [x(1); x(2); x(3); x(4); x(5); x(6)];
NMPC_ODIN.Jacobian.OutputFcn = @(x,u) [1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0];

%% Constraints---------------------------------------------------------------
for i = 1:6  % Input constraints
    NMPC_ODIN.MV(i).Min = -inf; 
    NMPC_ODIN.MV(i).Max = inf;
end

%% Cost Fcn------------------------------------------------------------------
NMPC_ODIN.Weights.OutputVariables = [5 5 5 1 1 1];

%% initialize states
x0 = zeros(12,1);  % Initial state
mv0 = zeros(6,1);  % Initial input

%% Closed-Loop Simulation
T_sim = 55;         % Total simulation time (seconds)
steps = T_sim / Ts; % Number of simulation steps

x = x0;            % Initialize state
mv = mv0;          % Initialize control input
u_history = [];    % Store control inputs
x_history = x0;    % Store state trajectory
error_hist = [];
yaw_hist   = [];
d = 1;
alpha = 1;
last_ref_idx = 1;  % Store the last value of ref_idx

%initialize obstacles
obs_coordinate = [1.5 1.5 1.5 0.3;
                  4 4.5 5 0.5 ;
                  6 4 6.3 0.5];

rho = zeros(size(obs_coordinate,1),1);
x_obs = zeros(size(obs_coordinate,1),1);
y_obs = zeros(size(obs_coordinate,1),1);
z_obs = zeros(size(obs_coordinate,1),1);
r_obs = zeros(size(obs_coordinate,1),1);
for l = 1:size(obs_coordinate, 1)
    x_obs(l) = obs_coordinate(l,1);
    y_obs(l) = obs_coordinate(l,2);
    z_obs(l) = obs_coordinate(l,3);
    r_obs(l) = obs_coordinate(l,4);
end

nloptions = nlmpcmoveopt; % Options for NLMPC solver
options = optimset('Display', 'off');

% Professional plotting setup
set(0, 'DefaultFigureRenderer', 'painters');
set(0, 'DefaultFigureColor', 'white');
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 12);
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultTextFontSize', 12);
set(0, 'DefaultLegendFontName', 'Times New Roman');
set(0, 'DefaultLegendFontSize', 11);

% Define professional color palette
colors = [
    0.00, 0.45, 0.74;  % Blue
    0.85, 0.33, 0.10;  % Red-orange
    0.93, 0.69, 0.13;  % Yellow-orange
    0.49, 0.18, 0.56;  % Purple
    0.47, 0.67, 0.19;  % Green
    0.30, 0.75, 0.93;  % Light blue
    0.64, 0.08, 0.18;  % Dark red
    0.74, 0.74, 0.74   % Gray
];

%% 1. Setup 3D Trajectory Figure
fig1 = figure('Position', [100, 100, 1000, 800]);
set(fig1, 'Color', 'white', 'PaperPositionMode', 'auto');

% Create axes with proper setup
ax1 = axes('Parent', fig1);
axis(ax1, 'equal'); 
grid(ax1, 'on');
hold(ax1, 'on');

% Set grid and axis properties
set(ax1, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax1, 'Box', 'on', 'LineWidth', 1.2);
set(ax1, 'FontSize', 12, 'FontName', 'Times New Roman');

% Labels and title
xlabel(ax1, 'X Position (m)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax1, 'Y Position (m)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
zlabel(ax1, 'Z Position (m)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
title(ax1, 'ODIN 3D Trajectory with Obstacle Avoidance', 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');

% Set initial view
view(ax1, 45, 30);

% Plot reference path
ref_plot     = plot3(ref_matrix(1,:),ref_matrix(2,:),ref_matrix(3,:),'r-','LineWidth',1);

% Initialize actual trajectory plot (will be updated in loop)
traj_plot = plot3(ax1, x(1), x(2), x(3), 'Color', colors(1,:), 'LineWidth', 3, ...
                  'DisplayName', 'Actual Trajectory');

% Initialize vehicle position marker
vehicle_mark = plot3(ax1, x(1), x(2), x(3), 'o', 'MarkerFaceColor', colors(1,:), ...
                     'MarkerEdgeColor', 'white', 'MarkerSize', 12, 'LineWidth', 2, ...
                     'DisplayName', 'Vehicle Position');

% obstacle visualization
[x_sphere, y_sphere, z_sphere] = sphere(30);
obstacle_handles = [];
for i = 1:size(obs_coordinate,1)
    % Create obstacle surface
    h_surf = surf(ax1, r_obs(i)*x_sphere + x_obs(i), ...
                  r_obs(i)*y_sphere + y_obs(i), ...
                  r_obs(i)*z_sphere + z_obs(i), ...
                  'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', colors(2,:));
    
    % Create obstacle center marker
    h_center = plot3(ax1, x_obs(i), y_obs(i), z_obs(i), 'x', 'Color', colors(2,:), ...
                     'MarkerSize', 12, 'LineWidth', 3);
    
    % Store handles for legend (only first obstacle for legend)
    if i == 1
        obstacle_handles = [h_surf, h_center];
    end
end

% Set axis limits for better visualization
xlim(ax1, [-1, 11]);
ylim(ax1, [-1, 13]);
zlim(ax1, [-1, 13]);

% Create comprehensive legend
legend_handles = [ref_plot, traj_plot, vehicle_mark, obstacle_handles(1)];
legend_labels = {'Reference Path', 'Actual Trajectory', 'Vehicle Position', 'Obstacles'};
leg1 = legend(ax1, legend_handles, legend_labels, 'Location', 'northeast', ...
              'FontSize', 11, 'Box', 'on', 'AutoUpdate', 'off');

%% 2. Setup Thruster Forces Figure (Fixed)
fig2 = figure('Position', [1100, 100, 900, 700]);
set(fig2, 'Color', 'white', 'PaperPositionMode', 'auto');

ax2 = axes('Parent', fig2);
hold(ax2, 'on'); 
grid(ax2, 'on');
set(ax2, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax2, 'Box', 'on', 'LineWidth', 1.2);
set(ax2, 'FontSize', 12, 'FontName', 'Times New Roman');

% Thruster labels
labels = ["Horizontal 1", "Horizontal 2", "Horizontal 3", "Horizontal 4", ...
          "Vertical 1", "Vertical 2", "Vertical 3", "Vertical 4"];
thr_plots = gobjects(8,1);

% Create thruster plots with professional colors and styling
for i = 1:8
    thr_plots(i) = plot(ax2, 0, 0, 'LineWidth', 2.5, 'Color', colors(mod(i-1,length(colors))+1,:), ...
                        'DisplayName', char(labels(i)));
end

xlabel(ax2, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax2, 'Thruster Force (N)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
title(ax2, 'Thruster Force Commands', 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');

% legend with two columns
legend(ax2, 'Location', 'eastoutside', 'FontSize', 10, 'NumColumns', 1, 'Box', 'on');

%% 3. Setup State Evolution Figure
fig3 = figure('Position', [200, 200, 1400, 1000]);
set(fig3, 'Color', 'white', 'PaperPositionMode', 'auto');

% Create subplots for state evolution
% Position states (x, y, z)
ax3_pos = subplot(2,2,1);
hold(ax3_pos, 'on');
grid(ax3_pos, 'on');
set(ax3_pos, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax3_pos, 'Box', 'on', 'LineWidth', 1.2);
set(ax3_pos, 'FontSize', 10, 'FontName', 'Times New Roman');
xlabel(ax3_pos, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax3_pos, 'Position (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
title(ax3_pos, 'Position States', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

% Linear velocity states (u, v, w)
ax3_vel = subplot(2,2,2);
hold(ax3_vel, 'on');
grid(ax3_vel, 'on');
set(ax3_vel, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax3_vel, 'Box', 'on', 'LineWidth', 1.2);
set(ax3_vel, 'FontSize', 10, 'FontName', 'Times New Roman');
xlabel(ax3_vel, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax3_vel, 'Linear Velocity (m/s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
title(ax3_vel, 'Linear Velocity States', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

% Orientation states (φ, θ, ψ)
ax3_ori = subplot(2,2,3);
hold(ax3_ori, 'on');
grid(ax3_ori, 'on');
set(ax3_ori, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax3_ori, 'Box', 'on', 'LineWidth', 1.2);
set(ax3_ori, 'FontSize', 10, 'FontName', 'Times New Roman');
xlabel(ax3_ori, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax3_ori, 'Orientation (rad)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
title(ax3_ori, 'Orientation States', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

% Angular velocity states (p, q, r)
ax3_angvel = subplot(2,2,4);
hold(ax3_angvel, 'on');
grid(ax3_angvel, 'on');
set(ax3_angvel, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax3_angvel, 'Box', 'on', 'LineWidth', 1.2);
set(ax3_angvel, 'FontSize', 10, 'FontName', 'Times New Roman');
xlabel(ax3_angvel, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax3_angvel, 'Angular Velocity (rad/s)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
title(ax3_angvel, 'Angular Velocity States', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');

% Initialize state plots
state_plots = cell(6,1);
state_plots{1} = gobjects(3,1); % Position
state_plots{2} = gobjects(3,1); % Linear velocity
state_plots{3} = gobjects(3,1); % Orientation
state_plots{4} = gobjects(3,1); % Angular velocity

% Position plots
state_plots{1}(1) = plot(ax3_pos, 0, 0, 'LineWidth', 2, 'Color', colors(1,:), 'DisplayName', 'x');
state_plots{1}(2) = plot(ax3_pos, 0, 0, 'LineWidth', 2, 'Color', colors(2,:), 'DisplayName', 'y');
state_plots{1}(3) = plot(ax3_pos, 0, 0, 'LineWidth', 2, 'Color', colors(3,:), 'DisplayName', 'z');
legend(ax3_pos, 'Location', 'best', 'FontSize', 9);

% Linear velocity plots
state_plots{2}(1) = plot(ax3_vel, 0, 0, 'LineWidth', 2, 'Color', colors(1,:), 'DisplayName', 'u');
state_plots{2}(2) = plot(ax3_vel, 0, 0, 'LineWidth', 2, 'Color', colors(2,:), 'DisplayName', 'v');
state_plots{2}(3) = plot(ax3_vel, 0, 0, 'LineWidth', 2, 'Color', colors(3,:), 'DisplayName', 'w');
legend(ax3_vel, 'Location', 'best', 'FontSize', 9);

% Orientation plots
state_plots{3}(1) = plot(ax3_ori, 0, 0, 'LineWidth', 2, 'Color', colors(1,:), 'DisplayName', 'φ (roll)');
state_plots{3}(2) = plot(ax3_ori, 0, 0, 'LineWidth', 2, 'Color', colors(2,:), 'DisplayName', 'θ (pitch)');
state_plots{3}(3) = plot(ax3_ori, 0, 0, 'LineWidth', 2, 'Color', colors(3,:), 'DisplayName', 'ψ (yaw)');
legend(ax3_ori, 'Location', 'best', 'FontSize', 9);

% Angular velocity plots
state_plots{4}(1) = plot(ax3_angvel, 0, 0, 'LineWidth', 2, 'Color', colors(1,:), 'DisplayName', 'p');
state_plots{4}(2) = plot(ax3_angvel, 0, 0, 'LineWidth', 2, 'Color', colors(2,:), 'DisplayName', 'q');
state_plots{4}(3) = plot(ax3_angvel, 0, 0, 'LineWidth', 2, 'Color', colors(3,:), 'DisplayName', 'r');
legend(ax3_angvel, 'Location', 'best', 'FontSize', 9);

% Number of obstacles
Nobs = size(obs_coordinate,1);

% Pre-allocate to store each rho(:,k)
rho_history = zeros(Nobs, steps);
thruster_history = zeros(8, steps); % Pre-allocate thruster history
min_rho_history = zeros(1, steps);

%% Main Simulation Loop
for k = 1:steps
    % Obstacle avoidance calculations
    for l = 1:size(obs_coordinate, 1)
        rho(l) = 0.5*(x(1)-x_obs(l))^2 + 0.5*(x(2)-y_obs(l))^2 + 0.5*(x(3)-z_obs(l))^2;
        obs_dis(l) = sqrt((x(1)-x_obs(l))^2 + (x(2)-y_obs(l))^2 + (x(3)-z_obs(l))^2);
    end
    
    [min_rho,~] = min(rho);
    [~,o] = min(rho);
    min_rho_history(k) = min_rho; 

    [min_obs_dis,~] = min(obs_dis);
    [~,j] = min(obs_dis);
    min_dis_history(k) = min_obs_dis; 
    
    rhopx = (x(1)-x_obs(o));
    rhopy = (x(2)-y_obs(o));
    rhopz = (x(3)-z_obs(o));
    dVx = alpha*d*(min_rho-d)*rhopx/((min_rho)^3);
    dVy = alpha*d*(min_rho-d)*rhopy/((min_rho)^3);
    dVz = alpha*d*(min_rho-d)*rhopz/((min_rho)^3);
    
    rho_history(:, k) = rho;

    % Reference tracking logic
    if isempty(last_ref_idx)
        last_ref_idx = 1;
    end

    if sqrt((x(1)-x_obs(o))^2 + (x(2)-y_obs(o))^2 + (x(3)-z_obs(o))^2) <= 2 + r_obs(o)
        new_ref_idx = min(last_ref_idx + 20, size(ref_matrix, 2));
    else
        new_ref_idx = min(last_ref_idx + 40, size(ref_matrix, 2));
    end

    ref_idx = max(last_ref_idx + 1, new_ref_idx);
    ref_idx = min(ref_idx, size(ref_matrix, 2));
    last_ref_idx = ref_idx;
    ref = transpose(ref_matrix(:, ref_idx));
    
    % NMPC control
    [mv,~,info] = nlmpcmove(NMPC_ODIN, x, mv, ref, [], nloptions);
    
    % Apply obstacle avoidance
    mv(1) = mv(1) - dVx;
    mv(2) = mv(2) - dVy;
    mv(3) = mv(3) - dVz;
    
    % Update state
    x = ODIN_dynamics(x, mv) * Ts + x;

    % Thruster allocation
    F = thursterallocation(mv);
    thruster_history(:,k) = F;
    
    % Store history
    u_history = [u_history mv]; 
    x_history = [x_history x];

    % Update 3D trajectory plot
    figure(fig1);
    set(traj_plot, 'XData', x_history(1,:), 'YData', x_history(2,:), 'ZData', x_history(3,:));
    set(vehicle_mark, 'XData', x(1), 'YData', x(2), 'ZData', x(3));

    % Update thruster plot
    figure(fig2);
    tvec = (1:k)*Ts;
    for i = 1:8
        set(thr_plots(i), 'XData', tvec, 'YData', thruster_history(i,1:k));
    end

     % Update state evolution plots
    figure(fig3);
    
    % Position states
    set(state_plots{1}(1), 'XData', tvec, 'YData', x_history(1,1:k));
    set(state_plots{1}(2), 'XData', tvec, 'YData', x_history(2,1:k));
    set(state_plots{1}(3), 'XData', tvec, 'YData', x_history(3,1:k));
    
    % Linear velocity states
    set(state_plots{2}(1), 'XData', tvec, 'YData', x_history(7,1:k));
    set(state_plots{2}(2), 'XData', tvec, 'YData', x_history(8,1:k));
    set(state_plots{2}(3), 'XData', tvec, 'YData', x_history(9,1:k));
    
    % Orientation states
    set(state_plots{3}(1), 'XData', tvec, 'YData', x_history(4,1:k));
    set(state_plots{3}(2), 'XData', tvec, 'YData', x_history(5,1:k));
    set(state_plots{3}(3), 'XData', tvec, 'YData', x_history(6,1:k));
    
    % Angular velocity states
    set(state_plots{4}(1), 'XData', tvec, 'YData', x_history(10,1:k));
    set(state_plots{4}(2), 'XData', tvec, 'YData', x_history(11,1:k));
    set(state_plots{4}(3), 'XData', tvec, 'YData', x_history(12,1:k));
    
    drawnow; 
    pause(0.05); % Reduced pause for smoother animation
    
    fprintf('Time: %.2f sec, Position: [%.2f, %.2f, %.2f], Min Distance: %.3f\n', ...
            k*Ts, x(1), x(2), x(3), sqrt(min_rho));
end

%% Barrier Function Plot
fig3 = figure('Position', [100, 450, 900, 600]);
set(fig3, 'Color', 'white', 'PaperPositionMode', 'auto');

ax3 = axes('Parent', fig3);
hold(ax3, 'on');

time = (1:steps)*Ts;
for i = 1:Nobs
    plot(ax3, time, rho_history(i,:), 'LineWidth', 2.5, 'Color', colors(i,:), ...
         'DisplayName', sprintf('Obstacle %d', i));
end

grid(ax3, 'on');
set(ax3, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax3, 'Box', 'on', 'LineWidth', 1.2);
set(ax3, 'FontSize', 12, 'FontName', 'Times New Roman');

xlabel(ax3, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax3, 'Barrier Function h(x)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
title(ax3, 'Evolution of Barrier Functions', 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');

legend(ax3, 'Location', 'best', 'FontSize', 11, 'Box', 'on');
ylim(ax3, [0, max(max(rho_history))*1.1]);

%% Minimum Barrier Function Plot
fig4 = figure('Position', [1100, 450, 900, 600]);
set(fig4, 'Color', 'white', 'PaperPositionMode', 'auto');

ax4 = axes('Parent', fig4);
hold(ax4, 'on');

plot(ax4, time, min_rho_history, 'LineWidth', 3, 'Color', colors(1,:), ...
     'DisplayName', 'Minimum Barrier Function');

grid(ax4, 'on');
set(ax4, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax4, 'Box', 'on', 'LineWidth', 1.2);
set(ax4, 'FontSize', 12, 'FontName', 'Times New Roman');

xlabel(ax4, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
title(ax4, 'Minimum Barrier Function', 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');

legend(ax4, 'Location', 'best', 'FontSize', 11, 'Box', 'on');
ylim(ax4, [0, max(min_rho_history)*1.1]);

% Add text annotation for minimum achieved distance
[min_val, min_idx] = min(min_rho_history);
text(ax4, time(min_idx), min_val, sprintf('Min: %.3f at t=%.1fs', min_val, time(min_idx)), ...
     'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

%% Nearest Obstacle Function Plot
fig4 = figure('Position', [1100, 450, 900, 600]);
set(fig4, 'Color', 'white', 'PaperPositionMode', 'auto');

ax4 = axes('Parent', fig4);
hold(ax4, 'on');

plot(ax4, time, min_dis_history, 'LineWidth', 3, 'Color', colors(1,:), ...
     'DisplayName', 'Distance to the nearest obstacle');

grid(ax4, 'on');
set(ax4, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);
set(ax4, 'Box', 'on', 'LineWidth', 1.2);
set(ax4, 'FontSize', 12, 'FontName', 'Times New Roman');

xlabel(ax4, 'Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax4, 'meter (m)', 'FontName', 'Times New Roman', 'FontSize', 14, 'FontWeight', 'bold');
title(ax4, 'Distance to the nearest obstacle', 'FontName', 'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold');

legend(ax4, 'Location', 'best', 'FontSize', 11, 'Box', 'on');
ylim(ax4, [0, max(min_dis_history)*1.1]);

%% Display final statistics
fprintf('\n=== SIMULATION COMPLETE ===\n');
fprintf('Total simulation time: %.1f seconds\n', T_sim);
fprintf('Final position: [%.2f, %.2f, %.2f]\n', x_history(1,end), x_history(2,end), x_history(3,end));
fprintf('Minimum distance to obstacles: %.3f\n', min(min_rho_history));
fprintf('Safety threshold: %.3f\n', d);
fprintf('Safety maintained: %s\n', ifelse(min(min_rho_history) > d, 'YES', 'NO'));

function F = thursterallocation(tau)
    s    = 1/sqrt(2);
    Rver = 0.252;
    Rhor = 0.356;
    E = [ s -s -s  s   0   0   0   0;
          s  s -s -s   0   0   0   0;
          0  0  0  0  -1  -1  -1  -1;
          0  0  0  0  Rver*s  Rver*s -Rver*s -Rver*s;
          0  0  0  0  Rver*s -Rver*s -Rver*s  Rver*s;
          Rhor -Rhor Rhor -Rhor 0      0      0      0 ];
    A   = [2*eye(8), E';
           E,       zeros(6)];
    sol = A \ [zeros(8,1); tau];
    F   = sol(1:8);
end

function result = ifelse(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end