function path = jumpPointSearch(start_pos, end_pos, obstacles, grid, gridStep)
    for row = 1:size(grid, 1)
        for col = 1:size(grid, 2)
            coords = matrix_coordinates(row, col, gridStep);
            obs = findClosestObstacle(coords(1,:), obstacles);
            distance_to_obs = pdist([coords(1,1),coords(1,2); obs(1),obs(2)],'euclidean');
            if distance_to_obs < 10
                grid(row, col) = 10;
            elseif distance_to_obs < 20
                grid(row, col) = 10; % Set the value at each position
            else
                grid(row, col) = 1; % Set the value at each position
            end
        end
    end

    % Define the start and goal positions
    startX = round(start_pos(1) / gridStep);
    startY = round(start_pos(2) / gridStep);
    
    % Check if the start position is within an obstacle
    if grid(startX, startY) > 1
        fprintf('Start position is within an obstacle. Value: %d\n', grid(startX, startY));
    end
    
    goalX = round(end_pos(1) / gridStep);
    goalY = round(end_pos(2) / gridStep);
    
    start = [startX, startY];
    goal = [goalX, goalY];
    
    % Perform Jump Point Search
    visited = false(size(grid));
    parent = zeros(size(grid));
    foundGoal = false;
    
    % Define the possible moves (up, down, left, right, diagonal)
    moves = [0, -1; 0, 1; -1, 0; 1, 0; -1, -1; -1, 1; 1, -1; 1, 1];
    
    % Initialize the queue with the start position
    queue = [start];
    
    while ~isempty(queue)
        current = queue(1,:);
        queue(1,:) = []; % Pop the front element
        
        if isequal(current, goal)
            foundGoal = true;
            break;
        end
        
        visited(current(1), current(2)) = true;
        neighbors = getNeighbors(current, grid, moves);
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i,:);
            row = neighbor(1);
            col = neighbor(2);
            
            if ~visited(row, col)
                queue = [queue; neighbor]; % Enqueue the neighbor
                visited(row, col) = true;
                parent(row, col) = sub2ind(size(grid), current(1), current(2));
                
                if isForced(current, neighbor, grid, moves)
                    jumpPoint = jump(neighbor, current, goal, grid, moves);
                    queue = [queue; jumpPoint]; % Enqueue the jump point
                end
            end
        end
    end
    
    % Retrieve the path if a goal was found
    if foundGoal
        path = reconstructPath(parent, goal, start);
        % pathCoords = path * gridStep;
        pathCoords = path(1:2:end, 1:2) * gridStep;
        checkpoints = [pathCoords(:, 1), pathCoords(:, 2); end_pos(1), end_pos(2)];
        path = checkpoints;
    else
        disp('No path found.');
        path = [];
    end
end

function neighbors = getNeighbors(current, grid, moves)
    neighbors = [];
    [maxRows, maxCols] = size(grid);
    
    for i = 1:size(moves, 1)
        move = moves(i,:);
        row = current(1) + move(1);
        col = current(2) + move(2);
        
        if row >= 1 && row <= maxRows && col >= 1 && col <= maxCols && grid(row, col) == 1
            neighbors = [neighbors; [row, col]];
        end
    end
end

function jumpPoint = jump(current, parent, goal, grid, moves)
    jumpPoint = current;
    [maxRows, maxCols] = size(grid);
    dx = current(1) - parent(1);
    dy = current(2) - parent(2);
    
    if ~isequal(current, goal)
        while true
            % Check if the current position is the goal or blocked
            if isGoal(current, goal)
                jumpPoint = current;
                return;
            elseif ~isClear(current, grid)
                return;
            end
            
            % Diagonal forced neighbor pruning
            if dx ~= 0 && dy ~= 0
                if (isClear([current(1) - dx, current(2)], grid) && ~isClear([current(1) - dx, current(2) + dy], grid)) || ...
                   (isClear([current(1), current(2) - dy], grid) && ~isClear([current(1) + dx, current(2) - dy], grid))
                    return;
                end
            end
            
            % Next jump point in the current direction
            next = current + [dx, dy];
            
            if ~isClear(next, grid)
                return;
            end
            
            % Check for forced neighbors
            if isForced(current, next, grid, moves)
                return;
            end
            
            current = next;
        end
    end
end

function path = reconstructPath(parent, goal, start)
    path = [];
    current = goal;
    
    while ~isequal(current, start)
        path = [current; path];
        index = parent(current(1), current(2));
        current = [mod(index-1, size(parent, 1))+1, floor((index-1)/size(parent, 1))+1];
    end
    
    path = [start; path];
end

function result = isClear(position, grid)
    row = position(1);
    col = position(2);
    
    if row >= 1 && row <= size(grid, 1) && col >= 1 && col <= size(grid, 2) && grid(row, col) == 1
        result = true;
    else
        result = false;
    end
end

function result = isGoal(position, goal)
    if position(1) == goal(1) && position(2) == goal(2)
        result = true;
    else
        result = false;
    end
end

function result = isForced(current, neighbor, grid, moves)
    dx = neighbor(1) - current(1);
    dy = neighbor(2) - current(2);
    
    if dx ~= 0 && dy ~= 0 % Diagonal move
        result = isClear(neighbor, grid) && (isClear([neighbor(1) - dx, neighbor(2)], grid) || isClear([neighbor(1), neighbor(2) - dy], grid));
    else % Straight move
        oppositeMove = -moves(moves(:,1) == -dx & moves(:,2) == -dy, :);
        oppositePos = current + oppositeMove;
        result = isClear(neighbor, grid) && isClear(oppositePos, grid);
    end
end

function h = heuristic(position, goal)
    % Euclidean distance heuristic
    h = norm(position - goal);
end

function coords = matrix_coordinates(row, col, gridStep)
    coords = [(row-1) * gridStep, (col-1) * gridStep];
end
