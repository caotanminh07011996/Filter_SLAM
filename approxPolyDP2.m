function [approxPoints, approxIndices] = approxPolyDP2(points, epsilon, indices)
    % approxPolyDP: Simplifies a polyline using the Ramer-Douglas-Peucker algorithm
    % points: Nx2 matrix containing the points [x, y]
    % epsilon: Simplification precision
    % indices: Optional Nx1 vector of original indices of the points
    
    % If indices are not provided, initialize as 1:N
    if nargin < 3
        indices = (1:size(points, 1))';
    end
    
    % If fewer than 3 points, return the points and indices as is
    if size(points, 1) < 3
        approxPoints = points;
        approxIndices = indices;
        return;
    end
    
    % Find the point with the maximum distance
    dmax = 0;
    index = 0;
    for i = 2:(size(points, 1) - 1)
        d = pointLineDist(points(i, :), points(1, :), points(end, :));
        if d > dmax
            index = i;
            dmax = d;
        end
    end
    
    % If the maximum distance is greater than epsilon, recursively simplify
    if dmax > epsilon
        % Recursive call for the first segment
        [recResults1, recIndices1] = approxPolyDP2(points(1:index, :), epsilon, indices(1:index));
        % Recursive call for the second segment
        [recResults2, recIndices2] = approxPolyDP2(points(index:end, :), epsilon, indices(index:end));
        
        % Combine the results, avoiding duplication of the middle point
        approxPoints = [recResults1(1:end-1, :); recResults2];
        approxIndices = [recIndices1(1:end-1); recIndices2];
    else
        % If not, return just the start and end points and their indices
        approxPoints = [points(1, :); points(end, :)];
        approxIndices = [indices(1); indices(end)];
    end
end
