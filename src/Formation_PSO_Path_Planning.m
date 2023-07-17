%% Communication-aware Formation Control
clear all;
close all;
clc;

% Initialize all parameters
max_iter    = 500;
h           = 1;
swarm_size  = 7;
alpha       = 10^(-5);                 % system parameter about antenna characteristics
delta       = 2;                       % required application data rate
beta        = alpha*(2^delta-1);
v           = 3;                       % path loss exponent
r0          = 5;                       % reference antenna near-field
PT          = 0.94;                    % reception probability threshold
rho_ij      = 0;
formation_speed = 1;
travel_speed = 0.5; % was 0.5
communication_qualities = zeros(swarm_size, swarm_size);
am         = 0.2; % was 0.2
bm         = 1;
a0         = 0.6; % was 0.6
b0         = 20; % was 20
bf         = 0;
af         = 1; % was 1
backstep_when_jammed = 1.2;
start_iteration = 20;
path_alg = "a_star";    % Options: a_star, d_star_lite, dijkstra


% The position of the destination
dest_x = 150;
dest_y = 150;
dest_pos = [dest_x, dest_y];

checkpoints = [dest_x, dest_y;];
checkpoint_index = 1;
checkpoint_threshold = 20; % Determines how close the centroid must be to complete a checkpoint
destination_threshold = 10;

% The position of the obstacle
obs_centers = [
    90, 90;]; % Coordinates of multiple obstacle centers

obs_radii = [
    35;
    ]; % Radii of multiple obstacles

avoid_directions = zeros(swarm_size);
swarm_obs = [];

stuckNum = 0;

%% ---Initialize Agents' Positions---
swarm = [
     7, 0;
    7, 30;
    0,   10;
    35,  20;
    58,   0;
    52,  13;
    52, 18;
    ];


%% ---Initialize the velocity---
for j = 1:swarm_size
    % speed(j,1) = rand();
    % speed(j,2) = rand();
    % prev_speed(j,1) = rand();
    % prev_speed(j,2) = rand();

    speed(j,1) = 0;
    speed(j,2) = 0;
    prev_speed(j,1) = 0;
    prev_speed(j,2) = 0;
end

% PSO Variables
personal_best_values = ones(swarm_size) * 10000;
global_best_value = 100000;

personal_best_locs = swarm;
global_best_loc = [0,0];

% PSO Constants
goal_w = 1;
obs_w = 100; % @65-80 for log scale
curr_vel_w = 0.11;
c1 = 0.07;
c2 = 0.13;

%% --- Grid Map Initialization --
% Define the size of the grid map and additional padding
padding = 100; % Additional padding in units
grid_step = 5; % Length of each grid

% Calculate the swarm centroid
centroid = mean(swarm);
% Define an empty array to store the centroid path
centroid_path = [];

grid_map = [];

%% ---Performance Indicators---
t_Elapsed = 0;
Jn        = 0;
Jn_arr = [];
rn        = 0;

% Define the figure positions
figure_positions = [
    %Left Bottom Right Width Height
    
    750, 480, 500, 400;    % Position for Figure 2
    200, 10, 500, 400;    % Position for Figure 3
    
    200, 480, 500, 400;   % Position for Figure 1
    750, 10, 500, 400;    % Position for Figure 4
    
    
    ];

% Plot Jn
figure(1)
Jn_Plot = plot(t_Elapsed, Jn);
set(gcf, 'Position', figure_positions(1, :));
xlabel('$t(s)$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
ylabel('$J_n$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
title('Average Communication Performance Indicator');
hold on
Jn_Text = text(t_Elapsed(end), Jn(end), sprintf('Jn: %.4f', Jn(end)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

% Plot rn
figure(2)
rn_Plot = plot(t_Elapsed, rn);
set(gcf, 'Position', figure_positions(2, :));
xlabel('$t(s)$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
ylabel('$r_n$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
title('Average Distance Indicator');
hold on
rn_Text = text(t_Elapsed(end), rn(end), sprintf('rn: %.4f', rn(end)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

drawnow                                         % Force a graphics refresh so that isn't counted in our elapsed time

tic

% Assign a different color to each edge-label pair
line_colors = rand(swarm_size, swarm_size, 3);
label_colors = line_colors;

% Define the color matrix with predefined colors
node_colors = [
    108 155 207;  % Light Blue
    247 147 39;   % Orange
    242 102 171;  % Light Pink
    122 168 116;  % Green
    255 217 90;   % Light Gold
    147 132 209;  % Purple
    245 80 80     % Red
    ] / 255;  % Divide by 255 to scale the RGB values to the [0, 1] range

[img, map, alphachannel] = imread('drone','png');
markersize = [3, 3];

%% ---Simulation---
for k=1:max_iter
    centroid = mean(swarm);
    centroid_path = [centroid_path; centroid(1), centroid(2)];
    if size(swarm_obs, 1) > 0
        % Calculate the minimum and maximum coordinates of the agents, obstacles, and destinations
        min_x = min([centroid(:, 1); swarm_obs(:, 1); dest_pos(:, 1)]) - padding;
        max_x = max([centroid(:, 1); swarm_obs(:, 1); dest_pos(:, 1)]) + padding;
        min_y = min([centroid(:, 2); swarm_obs(:, 2); dest_pos(:, 2)]) - padding;
        max_y = max([centroid(:, 2); swarm_obs(:, 2); dest_pos(:, 2)]) + padding;

        % Calculate the number of grid cells in each dimension
        num_cells_x = ceil((max_x - min_x) / grid_step);
        num_cells_y = ceil((max_y - min_y) / grid_step);

        % Initialize the matrix with zeros
        grid_map = zeros(num_cells_x, num_cells_y);

        % Create a meshgrid for the grid map
        [X, Y] = meshgrid(min_x:grid_step:max_x, min_y:grid_step:max_y);

        % Plot the grid map
        figure(5);
        clf;
        plot(X, Y, 'k-');
        hold on;
        
        % Checkpoints
        for c=1:size(checkpoints)
            dest_x = checkpoints(c, 1);
            dest_y = checkpoints(c, 2);
            if c < checkpoint_index
                fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'g');
            elseif c == size(checkpoints,1)
                fill([dest_pos(:, 1) - 2, dest_pos(:, 1) + 2, dest_pos(:, 1) + 2, dest_pos(:, 1) - 2, dest_pos(:, 1) - 2], [dest_pos(:, 2) - 2, dest_pos(:, 2) - 2, dest_pos(:, 2) + 2, dest_pos(:, 2) + 2, dest_pos(:, 2) - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'magenta');
            else
                fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2);
            end 
        end

        plot(X', Y', 'k-');
        axis equal;
        xlabel('$x$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
        ylabel('$y$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
        title('Grid Map');

        % Plot the agents, obstacles, and destinations on the grid map
        agent_handles = scatter(centroid(:, 1), centroid(:, 2), 'filled', 'MarkerFaceColor', 'b');
        obstacle_handles = scatter(swarm_obs(:, 1), swarm_obs(:, 2), 'filled', 'MarkerFaceColor', 'r');
        
        
        % Plot the centroid path as a yellow line
        plot(centroid_path(:, 1), centroid_path(:, 2), 'Color', 'y', 'LineWidth', 2);
        checkpoint_legend = plot(NaN, NaN, 's', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
        destination_handles = plot(NaN, NaN, 's', 'MarkerEdgeColor', 'magenta', 'LineWidth', 2);
        path_legend = plot(NaN, NaN, 'y', 'LineWidth', 2);
        
        % Add a legend
        lgd = legend([agent_handles, obstacle_handles, checkpoint_legend, destination_handles, path_legend], 'Swarm Centroid', 'Jam Location', 'Checkpoints', 'Destination', 'Centroid Path');
        lgd.Location = 'eastoutside';

        % Adjust the figure size to accommodate the legend
        pos = get(gcf, 'Position');
        pos(3) = pos(3) + 100; % Increase the width to accommodate the legend
        % set(gcf, 'Position', figure_positions(5, :));
    end

    % Plot the node trace inside the loop
    figure(4)
    clf;
    set(gcf, 'Position', figure_positions(4, :));
    xlabel('$x$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    ylabel('$y$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    title('Node Trace');
    hold on;
    axis equal;
    for j = 1:size(obs_centers,1)
        obs_center = obs_centers(j, :);
        obs_radius = obs_radii(j);
    
        % Plot the obstacle
        theta = linspace(0, 2*pi, 100);
        x = obs_radius * cos(theta) + obs_center(1);
        y = obs_radius * sin(theta) + obs_center(2);
        patch(x, y, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end

    % Plot all nodes as markers
    scatter(swarm(:, 1), swarm(:, 2), 16, node_colors, 'filled');

    %--- Formation Scene + Node Trace---
    for i = 1:swarm_size
        % Plot the node trace
        swarm_trace(k, i, :) = swarm(i, :);

        trace_x = squeeze(swarm_trace(:, i, 1));
        trace_y = squeeze(swarm_trace(:, i, 2));
        plot(trace_x, trace_y, 'Color', node_colors(i, :));

        % Add arrows to indicate node movement
        arrow_x = trace_x(1:end-1);
        arrow_y = trace_y(1:end-1);
        arrow_dx = diff(trace_x);
        arrow_dy = diff(trace_y);

        % Normalize the arrow displacements
        arrow_magnitudes = sqrt(arrow_dx.^2 + arrow_dy.^2);
        max_arrow_magnitude = max(arrow_magnitudes);
        scaling_factor = 2;  % Adjust the scaling factor for arrow size
        normalized_arrow_dx = arrow_dx * scaling_factor / max_arrow_magnitude;
        normalized_arrow_dy = arrow_dy * scaling_factor / max_arrow_magnitude;

        quiver(arrow_x, arrow_y, normalized_arrow_dx, normalized_arrow_dy, 0, 'Color', node_colors(i, :), 'LineWidth', 1.5, 'MaxHeadSize', 2);
       
        if size(swarm_obs, 1)
            plot(swarm_obs(:, 1), swarm_obs(:, 2), 'r*');
        end
    end

    %Destination
    for c=1:size(checkpoints)
        dest_x = checkpoints(c, 1);
        dest_y = checkpoints(c, 2);
        if c < checkpoint_index
            fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'g');
        elseif c == size(checkpoints,1)
            fill([dest_pos(:, 1) - 2, dest_pos(:, 1) + 2, dest_pos(:, 1) + 2, dest_pos(:, 1) - 2, dest_pos(:, 1) - 2], [dest_pos(:, 2) - 2, dest_pos(:, 2) - 2, dest_pos(:, 2) + 2, dest_pos(:, 2) + 2, dest_pos(:, 2) - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'magenta');
        else
            fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2);
        end 
    end

    figure(3);
    clf; % Clear the figure
    set(gcf, 'Position', figure_positions(3, :));
    xlabel('$x$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    ylabel('$y$', 'Interpreter','latex', 'FontSize', 12, 'Rotation', 0)
    title('Formation Scene');
    axis equal;
    hold on

    % Plot each obstacle
    for j = 1:size(obs_centers,1)
        obs_center = obs_centers(j, :);
        obs_radius = obs_radii(j);

        % Plot the obstacle
        theta = linspace(0, 2*pi, 100);
        x = obs_radius * cos(theta) + obs_center(1);
        y = obs_radius * sin(theta) + obs_center(2);
        patch(x, y, 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    
    hold on;
    %Destination
    for c=1:size(checkpoints)
        dest_x = checkpoints(c, 1);
        dest_y = checkpoints(c, 2);
        if c < checkpoint_index
            fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'g');
        elseif c == size(checkpoints,1)
            fill([dest_pos(:, 1) - 2, dest_pos(:, 1) + 2, dest_pos(:, 1) + 2, dest_pos(:, 1) - 2, dest_pos(:, 1) - 2], [dest_pos(:, 2) - 2, dest_pos(:, 2) - 2, dest_pos(:, 2) + 2, dest_pos(:, 2) + 2, dest_pos(:, 2) - 2], 'w', 'LineWidth', 2, 'EdgeColor', 'magenta');
        else
            fill([dest_x - 2, dest_x + 2, dest_x + 2, dest_x - 2, dest_x - 2], [dest_y - 2, dest_y - 2, dest_y + 2, dest_y + 2, dest_y - 2], 'w', 'LineWidth', 2);
        end 
    end
    
    if size(swarm_obs, 1)
        plot(swarm_obs(:, 1), swarm_obs(:, 2), 'r*');
        obs = findClosestObstacle(swarm(5, :), swarm_obs);
        plot(obs(1), obs(2), 'go');
    end

    plot(global_best_loc(1, 1), global_best_loc(1, 2), 'b*');

    
    for l = 1:swarm_size
        x_low = swarm(l, 1) - markersize(1)/2;
        x_high = swarm(l, 1) + markersize(1)/2;
        y_low = swarm(l, 2) - markersize(2)/2;
        y_high = swarm(l, 2) + markersize(2)/2;

        imagesc([x_low x_high], [y_low y_high], img, 'AlphaData', alphachannel, 'CData', repmat(reshape(node_colors(l, :), [1 1 3]), [size(img, 1), size(img, 2), 1]));
    end


    %--- Formation Scene Edge+Label ---
    for i = 1:swarm_size
        for j= 1:swarm_size
            if i ~= j && communication_qualities(i, j) > PT

                hold on;

                % Get the line color and label color for the current pair
                line_color = line_colors(i, j, :);
                label_color = label_colors(i, j, :);

                a1 = swarm(i,:);
                a2 = swarm(j,:);
                plot([a1(1), a2(1)], [a1(2), a2(2)], '--', 'Color', line_color);


                % Calculate the position for the label
                scaling_factor = 0.2;  % Adjust the scaling factor for the label positioning
                midpoint_x = (a1(1) + a2(1)) / 2;
                midpoint_y = (a1(2) + a2(2)) / 2;
                displacement_x = (a2(1) - a1(1)) * scaling_factor;
                displacement_y = (a2(2) - a1(2)) * scaling_factor;
                label_x = midpoint_x + displacement_x;
                label_y = midpoint_y + displacement_y + 2;

                % Add the number label at the midpoint of the line
                label = communication_qualities(i, j);
                label_str = sprintf('%.4f', label);

               % text(label_x, label_y, label_str, 'HorizontalAlignment', 'center', 'Color', label_color);
                hold off;

                % Remove quality value for refresh
                communication_qualities(i, j) = 0;
                communication_qualities(j, i) = 0;
            end
        end
    end

    hold off;

    axis equal;

    % STOP Simulation if reached destination
    if checkpoint_index > length(checkpoints)
        metric_iter_num = k - start_iteration;

        fprintf("Destination Reached\n");
        fprintf("END SIMULATION\n\n")
        fprintf("METRICS\n");
        fprintf("Num Iterations: %d\n", metric_iter_num);
        
        distances = zeros(swarm_size, 1);
        for d=start_iteration+1:k
            for j=1:swarm_size
                prev_coordinates = swarm_trace(d-1, j, :);
                curr_coordinates = swarm_trace(d, j, :);

                x = [prev_coordinates(1), prev_coordinates(2); curr_coordinates(1), curr_coordinates(2);];
                distance = pdist(x,'euclidean');
                distances(j) = distances(j) + distance;
            end
        end

        efficiencies = distances(:) / metric_iter_num;
        mean_dist = mean(distances);
        disp(distances);
        fprintf("Avg. Distance: %f\n", mean_dist);
        fprintf("Avg. Efficiency: %f\n", mean(efficiencies));
        fprintf("Avg. Jn: %f\n", mean(Jn_arr));
        return;
    end

    %--- Controller ---
    for i=1:swarm_size
        rho_ij=0;
        phi_rij=0;
        rij=0;

        % Gradient Controller
        for j=[1:(i-1),(i+1):swarm_size]
            rij=sqrt((swarm(i,1)-swarm(j,1))^2+(swarm(i,2)-swarm(j,2))^2);
            aij=exp(-alpha*(2^delta-1)*(rij/r0)^v);
            gij=rij/sqrt(rij^2+r0^2);

            % Check if agent is within a certain range of any of the obstacles
            for m = 1:size(obs_centers,1)
                obs_center = obs_centers(m, :);
                obs_radius = obs_radii(m);
    
                if norm(swarm(i, :) - obs_center) <= obs_radius + b0
                    % Check if communication link is in Jam Area
                    if lineCrossCircle(swarm(i, :), swarm(j, :), obs_center, obs_radius)
                        % Half the quality values when jammed
                        aij = aij * 0.5;
                        gij = gij * 0.5;
                    end
                end
            end

            if aij>=PT
                rho_ij=(-beta*v*rij^(v+2)-beta*v*(r0^2)*(rij^v)+r0^(v+2))*exp(-beta*(rij/r0)^v)/sqrt((rij^2+r0^2)^3);
            else
                rho_ij=0;
            end

            phi_rij=gij*aij;
            communication_qualities(i,j) = phi_rij;
            qi=[swarm(i,1),swarm(i,2)];
            qj=[swarm(j,1),swarm(j,2)];
            nd=(qi-qj)/sqrt(1+norm(qi-qj)*formation_speed);

            speed(i,1)=speed(i,1)+rho_ij * nd(1) * travel_speed;
            speed(i,2)=speed(i,2)+rho_ij * nd(2) * travel_speed;
            
        end

        % PSO Controller
        if k >= start_iteration
            for l=1:swarm_size
                X = [checkpoints(checkpoint_index, 1),checkpoints(checkpoint_index, 2); swarm(l, 1),swarm(l, 2)];
                distance_to_goal = pdist(X,'euclidean');
                proximity_to_obstacle = 0;
                
                if size(swarm_obs) > 0
                    obs = findClosestObstacle(swarm(l, :), swarm_obs);
                    X = [obs(1),obs(2); swarm(l, 1),swarm(l, 2)];
                    proximity_to_obstacle = pdist(X,'euclidean');
                    % fprintf("Proximity: %f \n", proximity_to_obstacle);
                    if proximity_to_obstacle > 70
                       proximity_to_obstacle = 70;
                    end
                    proximity_to_obstacle = log(proximity_to_obstacle) / log(10);
                end
                
                % Evaluate Agent Fitness  
                fitness = goal_w * distance_to_goal - obs_w * proximity_to_obstacle;
                
                % Update personal best fitness
                if fitness < personal_best_values(l)
                    personal_best_values(l) = fitness;
                    personal_best_locs(l, :) = swarm(l, :);
                end
                
                % Update global best fitness
                if fitness < global_best_value
                    global_best_value = fitness;
                    global_best_loc(1, :) = swarm(l, :);
                    % fprintf("Global best updated\n")
                end
            end
            
            % Velocities with Random values
            % new_velocity_x = curr_vel_w * prev_speed(i, 1) + c1 * rand() * (personal_best_locs(i, 1) - swarm(i, 1)) + c2 * rand() * (global_best_loc(1, 1) - swarm(i, 1));
            % new_velocity_y = curr_vel_w * prev_speed(i, 2) + c1 * rand() * (personal_best_locs(i, 2) - swarm(i, 2)) + c2 * rand() * (global_best_loc(1, 2) - swarm(i, 2));
            
            % Without random values
            new_velocity_x = curr_vel_w * prev_speed(i, 1) + c1  * (personal_best_locs(i, 1) - swarm(i, 1)) + c2  * (global_best_loc(1, 1) - swarm(i, 1));
            new_velocity_y = curr_vel_w * prev_speed(i, 2) + c1  * (personal_best_locs(i, 2) - swarm(i, 2)) + c2  * (global_best_loc(1, 2) - swarm(i, 2));
            
            % Add values to speed
            speed(i, 1) = speed(i, 1) + new_velocity_x;
            speed(i, 2) = speed(i, 2) + new_velocity_y;
        end

        % Agent has no communication links
        if all(communication_qualities(i, :) < PT) 
            swarm_obs = [swarm_obs; swarm(i, :)];
            fprintf("New Obstacle found: %d\n", size(swarm_obs, 1));

            % Reset PSO best values
            global_best_value = 10000;
            personal_best_values = ones(swarm_size) * 10000;
            speed(i,:) = -prev_speed(i,:) * backstep_when_jammed;

            min_x = min([centroid(:, 1); swarm_obs(:, 1); dest_pos(:, 1)]) - padding;
            max_x = max([centroid(:, 1); swarm_obs(:, 1); dest_pos(:, 1)]) + padding;
            min_y = min([centroid(:, 2); swarm_obs(:, 2); dest_pos(:, 2)]) - padding;
            max_y = max([centroid(:, 2); swarm_obs(:, 2); dest_pos(:, 2)]) + padding;
    
            % Calculate the number of grid cells in each dimension
            num_cells_x = ceil((max_x - min_x) / grid_step);
            num_cells_y = ceil((max_y - min_y) / grid_step);
    
            % fprintf("Num cells: %d\n", num_cells_x)
    
            % Initialize the matrix with zeros
            grid_map = zeros(num_cells_x, num_cells_y);

            % Recalulate path
            path =[];
            if path_alg == "a_star"
                path = aStar(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            elseif path_alg == "d_star_lite"
                path = dStarLite(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            elseif path_alg == "dijkstra"
                path = dijkstra(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            end

            if size(path) > 0
                checkpoints = path;
                checkpoint_index = 1;
            end
            
        end

        % Update Agent positions
        swarm(i,:)=swarm(i,:)+speed(i,:)*h;

        % Record agents last velocity
        prev_speed(i,:) = speed(i,:)*h;
        
        speed(i,:)=0;

        t_Elapsed=cat(1, t_Elapsed, toc);

        %---Average Communication Performance Indicator---%
        Jn=cat(1, Jn, phi_rij);
        
        Jn=smooth(Jn);

        if k >= start_iteration
            Jn_arr = [Jn_arr; phi_rij];
        end
        set(Jn_Plot, 'xdata', t_Elapsed, 'ydata', Jn);      % Plot Jn
        set(Jn_Text, 'Position', [t_Elapsed(end), Jn(end)], 'String', sprintf('Jn: %.4f', Jn(end)));
        pause(0);

        %---Average Neighboring Distance Indicator---%
        rn=cat(1, rn, rij);
        rn=smooth(rn);
        set(rn_Plot, 'xdata', t_Elapsed, 'ydata', rn);      % Plot rn
        set(rn_Text, 'Position', [t_Elapsed(end), rn(end)], 'String', sprintf('rn: %.4f', rn(end)));

        pause(0);

    end

    % Update checkpoint
    centroid = mean(swarm);
    dist_to_checkpoint = norm([checkpoints(checkpoint_index, 1) - centroid(1), checkpoints(checkpoint_index, 2) - centroid(2)]);
    checked = false;
    threshold = checkpoint_threshold;
    if checkpoint_index == size(checkpoints,1)
        threshold = destination_threshold;
    end
    
    if dist_to_checkpoint < threshold
        fprintf("Checkpoint %d reached\n", checkpoint_index);
        checkpoint_index =  checkpoint_index + 1;
        % fprintf("Size %f\n", size(checkpoints,1));
        if checkpoint_index < size(checkpoints,1)
            dist_to_next_checkpoint = norm([checkpoints(checkpoint_index, 1) - centroid(1), checkpoints(checkpoint_index, 2) - centroid(2)]);
            % fprintf("IN DOUBLE CHECK");
            if dist_to_next_checkpoint < threshold
                fprintf("Checkpoint %d reached\n", checkpoint_index);
                % fprintf("DOUBLE CHECK\n");
                checkpoint_index =  checkpoint_index + 1;
            end
        end
        % Reset PSO best values
        global_best_value = 10000;
        personal_best_values = ones(swarm_size) * 10000;
        checked = true;
    end

    dist_to_gbest = norm([global_best_loc(1, 1) - centroid(1), global_best_loc(1, 2) - centroid(2)]);
    if dist_to_gbest < 5 && size(checkpoints,1) > 1
        
        % Recalulate path
        if stuckNum == 0
            path =[];
            if path_alg == "a_star"
                path = aStar(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            elseif path_alg == "d_star_lite"
                path = dStarLite(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            elseif path_alg == "dijkstra"
                path = dijkstra(mean(swarm), [dest_x dest_y], swarm_obs, grid_map, grid_step);
            end
            
            if size(path) > 0
                checkpoints = path;
                checkpoint_index = 1;
                global_best_value = 10000;
                personal_best_values = ones(swarm_size) * 10000;
            end
            fprintf("Recalulate path cause stuck\n", mean(prev_speed))
        end
        
        
        if mod(stuckNum, 5) == 0 && checkpoint_index < size(checkpoints,1)
            fprintf("Update checkpoint cause stuck, stuck num: %d\n", stuckNum)
            checkpoint_index = checkpoint_index + 1;
            global_best_value = 10000;
            personal_best_values = ones(swarm_size) * 10000;
            goal_w = 10;
            % for j = 1:swarm_size
            %     prev_speed(j,1) = rand() * 5;
            %     prev_speed(j,2) = rand() * 5;
            % end
            
        end
        stuckNum = stuckNum + 1;
    
        
    else
        stuckNum = 0;
        goal_w = 1;
    end

    
    pause(0)

end

figure(1)
axis([0 300  0, 1]);  % Update the limits for Figure 1

figure(2)
axis([0 300 min(rn) max(rn)+5]);  % Update the limits for Figure 2

% Plot the final node trace outside the loop
figure(4)
hold on;
for i = 1:swarm_size
    trace_x = squeeze(swarm_trace(:, i, 1));
    trace_y = squeeze(swarm_trace(:, i, 2));
    plot(trace_x, trace_y);
end
hold off;