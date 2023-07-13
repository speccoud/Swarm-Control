function path = aStar(start_pos, end_pos, obstacles)
    gridSizeX = 500;
    gridSizeY = 500;
    gridStep = 5;

    grid = zeros(gridSizeX/gridStep, gridSizeY/gridStep); % Initialize the matrix with zeros

    for row = 1:size(grid, 1)
        for col = 1:size(grid, 2)
            coords = matrix_coordinates(row, col, gridStep);
            obs = findClosestObstacle(coords(1,:), obstacles);
            distance_to_obs = pdist([coords(1,1),coords(1,2); obs(1),obs(2)],'euclidean');
            if distance_to_obs < 10
                grid(row, col) = 10;
            elseif distance_to_obs < 20
                grid(row, col) = 10; % Set the value at each position
            elseif distance_to_obs < 25
                grid(row, col) = 1; % Set the value at each position
            else
                grid(row, col) = 1; % Set the value at each position
            end
            
        end
    end

    visited = false(size(grid));
    % Define the start and goal positions
    startX = round(start_pos(1) / gridStep);
    startY = round(start_pos(2) / gridStep);
    
    % Add a check to ensure start indices are within range
    startX = max(1, min(startX, size(visited, 1)));
    startY = max(1, min(startY, size(visited, 2)));

     % Check if the start position is within an obstacle
    if grid(startX, startY) > 1
        fprintf('Start position is within an obstacle. Value: %d\n', grid(startX, startY));
        % path = [];
        % return;
    end
    
    goalX = round(end_pos(1) / gridStep);
    goalY = round(end_pos(2) / gridStep);
    
    % Add a check to ensure goal indices are within range
    goalX = max(1, min(goalX, size(visited, 1)));
    goalY = max(1, min(goalY, size(visited, 2)));
    % fprintf("StartX: %f, ", startX)
    % fprintf("Y: %f\n", startY)
    % fprintf("End: %f, ", goalX)
    % fprintf("Y: %f\n", goalY)

    start = [startX, startY];
    goal = [goalX, goalY];
    
    % Perform A* search
    queue = PriorityQueue();
    queue.insert(start, 0); % Start position with priority 0
    visited(start(1), start(2)) = true;
    parent = zeros(size(grid));
    gScore = inf(size(grid));
    gScore(start(1), start(2)) = 0;
    foundGoal = false;
    
    % Define the possible moves (up, down, left, right, diagonal)
    moves = [0, -1; 0, 1; -1, 0; 1, 0; -1, -1; -1, 1; 1, -1; 1, 1];
    
    while ~queue.isEmpty()
        current = queue.pop();
        
        if current(1) == goal(1) && current(2) == goal(2)
            foundGoal = true;
            break;
        end
        
        for i = 1:size(moves, 1)
            next = current + moves(i, :);
            row = next(1);
            col = next(2);
            
            % Check if the next position is valid and not visited
            if row >= 1 && row <= size(grid, 1) && col >= 1 && col <= size(grid, 2) && grid(row, col) == 1 && ~visited(row, col)
                visited(row, col) = true;
                newGScore = gScore(current(1), current(2)) + 1; % Assuming a uniform cost of 1 for each step
                if newGScore < gScore(row, col)
                    gScore(row, col) = newGScore;
                    priority = newGScore + heuristic(next, goal);
                    queue.insert(next, priority);
                    parent(row, col) = sub2ind(size(grid), current(1), current(2));
                end
            end
        end
    end
    
    % Retrieve the path if a goal was found
    if foundGoal
        path = [];
        current = goal;
        while ~isequal(current, start)
            path = [current; path];
            index = parent(current(1), current(2));
            current = [mod(index-1, size(grid, 1))+1, floor((index-1)/size(grid, 1))+1];
        end
        path = [start; path];
        
        % Convert path positions to indexes
        indexes = sub2ind(size(grid), path(:, 1), path(:, 2));
        
        % disp('Indexes of the cells in the path:');
        % disp(indexes);

        % Convert path positions to 2D coordinates
        pathCoords = path(:, 1:2) * gridStep;
        checkpoints = [pathCoords(:, 1), pathCoords(:, 2); end_pos(1), end_pos(2)];
        
        % disp('Coordinates of the cells in the path:');
        % disp(checkpoints);
        path = checkpoints;
    else
        disp('No path found.');
        path = [];
    end
end

function h = heuristic(position, goal)
    % Euclidean distance heuristic
    h = norm(position - goal);
end

function coords = matrix_coordinates(row, col, gridStep)
    coords = [(row-1) * gridStep, (col-1) * gridStep];
end
