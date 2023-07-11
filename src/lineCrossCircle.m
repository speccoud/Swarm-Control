function isCrossing = lineCrossCircle(lineStart, lineEnd, circleCenter, circleRadius)
    % Extract the coordinates of the line and circle
    x1 = lineStart(1);
    y1 = lineStart(2);
    x2 = lineEnd(1);
    y2 = lineEnd(2);
    circleX = circleCenter(1);
    circleY = circleCenter(2);
    
    % Check if any of the line endpoints are inside the circle
    if (x1 - circleX)^2 + (y1 - circleY)^2 <= circleRadius^2 || ...
       (x2 - circleX)^2 + (y2 - circleY)^2 <= circleRadius^2
        isCrossing = true;
        return;
    end
    
    % Check if the line intersects with the circle
    dx = x2 - x1;
    dy = y2 - y1;
    dr = sqrt(dx^2 + dy^2);
    D = x1 * y2 - x2 * y1;
    discriminant = circleRadius^2 * dr^2 - D^2;
    
    if isnan(dx) || isnan(dy) || isnan(dr) || isnan(D) || isnan(discriminant)
        isCrossing = false;
        return;
    end
    
    if discriminant >= 0
        x_intersect1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / (dr^2);
        y_intersect1 = (-D * dx + abs(dy) * sqrt(discriminant)) / (dr^2);
        x_intersect2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / (dr^2);
        y_intersect2 = (-D * dx - abs(dy) * sqrt(discriminant)) / (dr^2);
        
        % Check if the intersection points are within the line segment
        if isPointOnSegment(x1, y1, x2, y2, x_intersect1, y_intersect1) || ...
           isPointOnSegment(x1, y1, x2, y2, x_intersect2, y_intersect2)
            isCrossing = true;
            return;
        end
    end
    
    % If none of the above conditions are met, the line does not cross the circle
    isCrossing = false;
end

function isOnSegment = isPointOnSegment(x1, y1, x2, y2, x3, y3)
    if isnan(x1) || isnan(y1) || isnan(x2) || isnan(y2) || isnan(x3) || isnan(y3)
        isOnSegment = false;
        return;
    end
    
    if x3 >= min(x1, x2) && x3 <= max(x1, x2) && y3 >= min(y1, y2) && y3 <= max(y1, y2)
        isOnSegment = true;
    else
        isOnSegment = false;
    end
end
