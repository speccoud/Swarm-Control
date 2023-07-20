function path = thetaStar(start_pos, end_pos, obstacles, grid, gridStep)

    for row = 1:size(grid, 1)
        for col = 1:size(grid, 2)
            coords = matrix_coordinates(row, col, gridStep);
            obs = findClosestObstacle(coords(1,:), obstacles);
            distance_to_obs = pdist([coords(1,1),coords(1,2); obs(1),obs(2)],'euclidean');
            if distance_to_obs < 15
                grid(row, col) = 10;
            elseif distance_to_obs < 25
                grid(row, col) = 5; % Set the value at each position
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
    % if grid(startX, startY) > 1
    %     disp('Start position is inside an obstacle.');
    %     path = [];
    %     return;
    % end
    
    goalX = round(end_pos(1) / gridStep);
    goalY = round(end_pos(2) / gridStep);
    
    % Add a check to ensure goal indices are within range
    goalX = max(1, min(goalX, size(visited, 1)));
    goalY = max(1, min(goalY, size(visited, 2)));

    start = [startX, startY];
    goal = [goalX, goalY];
    
    % Perform Theta* search
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
        
        % Theta* improvement: check visibility before exploring the neighbor
        for i = 1:size(moves, 1)
            next = current + moves(i, :);
            row = next(1);
            col = next(2);
            
            % Check if the next position is within the grid bounds
            if row >= 1 && row <= size(grid, 1) && col >= 1 && col <= size(grid, 2)
                % Check if the neighbor is visible (no obstacle in between)
                if ~lineOfSight(current, next, grid, obstacles)
                    continue; % Skip to the next neighbor
                end

                % The neighbor is visible, proceed with processing
                if grid(row, col) == 1 && ~visited(row, col)
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
        pathCoords = path(1:2:end, 1:2) * gridStep;
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

function visible = lineOfSight(start, goal, grid, obstacles)
    % Check if there is a line of sight between start and goal positions
    visible = true;

    % Compute the line of sight between the start and goal
    points = bresenham(start(1), start(2), goal(1), goal(2));

    % Check each point along the line
    for i = 1:size(points, 1)
        x = points(i, 1);
        y = points(i, 2);

        % Check if the point is outside the grid
        if x < 1 || x > size(grid, 1) || y < 1 || y > size(grid, 2)
            visible = false;
            break;
        end

        % Check if the point is inside an obstacle
        if grid(x, y) > 1 || isPointInsideObstacle([x, y], obstacles)
            visible = false;
            break;
        end
    end
end

function inside = isPointInsideObstacle(point, obstacles)
    % Check if the given point is inside any of the obstacles
    inside = false;

    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        if inpolygon(point(1), point(2), obstacle(:, 1), obstacle(:, 2))
            inside = true;
            break;
        end
    end
end

function points = bresenham(x1, y1, x2, y2)
    % Bresenham's line algorithm to compute points along a line
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    steep = (dy > dx);

    if steep
        [x1, y1] = swap(x1, y1);
        [x2, y2] = swap(x2, y2);
    end

    if x1 > x2
        [x1, x2] = swap(x1, x2);
        [y1, y2] = swap(y1, y2);
    end

    dx = x2 - x1;
    dy = abs(y2 - y1);

    error = dx / 2;
    ystep = 1;
    if y1 >= y2
        ystep = -1;
    end

    y = y1;
    points = zeros(dx, 2);
    for x = x1:x2
        if steep
            points(x - x1 + 1, :) = [y, x];
        else
            points(x - x1 + 1, :) = [x, y];
        end

        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end
end

function [a, b] = swap(a, b)
    temp = a;
    a = b;
    b = temp;
end
