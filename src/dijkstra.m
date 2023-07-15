function path = dijkstra(start_pos, end_pos, obstacles, grid, gridStep)
    for row = 1:size(grid, 1)
        for col = 1:size(grid, 2)
            coords = matrix_coordinates(row, col, gridStep);
            obs = findClosestObstacle(coords(1,:), obstacles);
            distance_to_obs = pdist([coords(1,1),coords(1,2); obs(1),obs(2)],'euclidean');
            if distance_to_obs < 10
                grid(row, col) = 10;
            elseif distance_to_obs < 20
                grid(row, col) = 5; % Set the value at each position
            else
                grid(row, col) = 1; % Set the value at each position
            end
        end
    end

    visited = false(size(grid));
    startX = round(start_pos(1) / gridStep);
    startY = round(start_pos(2) / gridStep);
    
    startX = max(1, min(startX, size(visited, 1)));
    startY = max(1, min(startY, size(visited, 2)));

    if grid(startX, startY) > 1
        fprintf('Start position is within an obstacle. Value: %d\n', grid(startX, startY));
    end
    
    goalX = round(end_pos(1) / gridStep);
    goalY = round(end_pos(2) / gridStep);
    
    goalX = max(1, min(goalX, size(visited, 1)));
    goalY = max(1, min(goalY, size(visited, 2)));

    start = [startX, startY];
    goal = [goalX, goalY];
    
    queue = PriorityQueue();
    queue.insert(goal, 0);
    visited(goal(1), goal(2)) = true;
    distance = inf(size(grid));
    distance(goal(1), goal(2)) = 0;
    
    % Define the possible moves (up, down, left, right, diagonal)
    moves = [0, -1; 0, 1; -1, 0; 1, 0; -1, -1; -1, 1; 1, -1; 1, 1];
    
    while ~queue.isEmpty()
        current = queue.pop();
        
        if ~isinf(distance(current(1), current(2)))
            for i = 1:size(moves, 1)
                next = current + moves(i, :);
                row = next(1);
                col = next(2);

                if row >= 1 && row <= size(grid, 1) && col >= 1 && col <= size(grid, 2)
                    if grid(row, col) == 1
                        newDistance = distance(current(1), current(2)) + 1;
                        if newDistance < distance(row, col)
                            distance(row, col) = newDistance;
                            if ~visited(row, col)
                                visited(row, col) = true;
                                queue.insert([row, col], newDistance);
                            end
                        end
                    end
                end
            end
        end
    end
    
    % Retrieve the path if a goal was found
    if visited(startX, startY)
        path = [];
        current = start;
        while ~isequal(current, goal)
            path = [current; path];
            neighbors = getNeighbors(current, moves);
            minVal = inf;
            minVertex = current;
            
            for i = 1:size(neighbors, 1)
                neighbor = neighbors(i, :);
                if distance(neighbor(1), neighbor(2)) + 1 < minVal
                    minVal = distance(neighbor(1), neighbor(2)) + 1;
                    minVertex = neighbor;
                end
            end
            
            current = minVertex;
        end
        path = [path; goal];
        
        % Convert path positions to indexes
        indexes = sub2ind(size(grid), path(:, 1), path(:, 2));
        
        % Convert path positions to 2D coordinates
        pathCoords = path(1:2:end, 1:2) * gridStep;
        checkpoints = [pathCoords(:, 1), pathCoords(:, 2); end_pos(1), end_pos(2)];
        
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

function key = calculateKey(vertex, rhs, gScore, km)
    key = [min(gScore(vertex(1), vertex(2)), rhs(vertex(1), vertex(2))) + heuristic(vertex, goal) + km, min(gScore(vertex(1), vertex(2)), rhs(vertex(1), vertex(2)))];
end

function updateVertex(vertex, start, goal, rhs, gScore, grid, km, queue, visited)
    if ~isequal(vertex, goal)
        neighbors = getNeighbors(vertex, moves);
        minVal = inf;
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if gScore(neighbor(1), neighbor(2)) + 1 < minVal
                minVal = gScore(neighbor(1), neighbor(2)) + 1;
            end
        end
        
        rhs(vertex(1), vertex(2)) = minVal;
    end
    
    if visited(vertex(1), vertex(2))
        queue.insert(vertex, calculateKey(vertex, rhs, gScore, km));
    end
end

function neighbors = getNeighbors(vertex, moves)
    neighbors = vertex + moves;
end