function checkpoints = aStar(start_pos, end_pos, obstacles)
    % Initialize the grid map and node information
    grid_size = 500; % Adjust the grid size based on your requirements
    num_agents = size(start_pos, 1);
    start_node = Node(start_pos, 0, heuristic(start_pos, end_pos), 0);
    goal_node = Node(end_pos, 0, 0, 0);
    open_set = PriorityQueue();
    open_set.enqueue(start_node);
    closed_set = [];
    
    % Create the grid map with obstacle flags
    grid_map = zeros(grid_size, grid_size);
    for i = 1:size(obstacles, 1)
        obs_x = obstacles(i, 1);
        obs_y = obstacles(i, 2);
        grid_map(obs_y, obs_x) = 1; % Mark obstacle cells as 1
    end
    
    % Define the neighbors' movement directions
    neighbors = [0, 1; 0, -1; 1, 0; -1, 0; 1, 1; 1, -1; -1, 1; -1, -1];
    
    % A* algorithm
    while ~open_set.isEmpty()
        current_node = open_set.dequeue();
        
        % Check if goal reached
        if isGoal(current_node, goal_node)
            checkpoints = reconstructPath(current_node);
            return;
        end
        
        closed_set = [closed_set; current_node];
        
        % Generate neighboring nodes
        for i = 1:size(neighbors, 1)
            neighbor_pos = current_node.position + neighbors(i, :);
            
            % Skip if neighbor is out of grid bounds or an obstacle
            if ~isValidPosition(neighbor_pos, grid_size) || isObstacle(neighbor_pos, grid_map)
                continue;
            end
            
            neighbor_node = Node(neighbor_pos, current_node.g + 1, heuristic(neighbor_pos, end_pos), current_node);
            
            % Check if neighbor already evaluated
            if isNodeEvaluated(neighbor_node, closed_set)
                continue;
            end
            
            % Calculate tentative g-score
            tentative_g_score = current_node.g + 1;
            
            % Check if neighbor is already in the open set
            if ~isNodeEvaluated(neighbor_node, open_set)
                open_set.enqueue(neighbor_node);
            elseif tentative_g_score >= neighbor_node.g
                continue;
            end
            
            neighbor_node.g = tentative_g_score;
            neighbor_node.f = neighbor_node.g + neighbor_node.h;
            neighbor_node.parent = current_node;
        end
    end
    
    % No path found
    checkpoints = [];
    disp("No path found!");
end

function h = heuristic(pos1, pos2)
    % Euclidean distance heuristic
    h = norm(pos2 - pos1);
end

function valid = isValidPosition(position, grid_size)
    % Check if position is within grid bounds
    valid = all(position >= 1) && all(position <= grid_size);
end

function obstacle = isObstacle(position, grid_map)
    % Check if position is an obstacle
    obstacle = grid_map(position(2), position(1)) == 1;
end

function goal = isGoal(node, goal_node)
    % Check if node is the goal
    goal = all(node.position == goal_node.position);
end

function evaluated = isNodeEvaluated(node, node_list)
    % Check if node is already evaluated
    evaluated = any(arrayfun(@(n) all(n.position == node.position), node_list));
end

function path = reconstructPath(node)
    % Reconstruct the path from goal to start
    path = [];
    while ~isempty(node)
        path = [node.position; path];
        node = node.parent;
    end
end