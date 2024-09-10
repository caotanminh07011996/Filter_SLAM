function [R, t, transformedPoints] = ICPPointToLine(points, segments, maxIterations, tolerance)
    % ICPPointToLine: Performs the ICP algorithm to align 'points' to 'segments'.
    % points: Nx2 matrix of points to be aligned [x, y]
    % segments: Struct array where each struct contains fields:
    %   - startPoint: 1x2 vector [x1, y1]
    %   - endPoint: 1x2 vector [x2, y2]
    % maxIterations: Maximum number of iterations for the ICP algorithm
    % tolerance: Convergence tolerance based on the change in error
    
    % Initialize transformation (rotation R and translation t)
    R = eye(2); % 2x2 identity matrix
    t = zeros(1, 2); % 1x2 zero vector
    
    % Initialize transformed points as the original points
    transformedPoints = points;
    
    % Iterate for the maximum number of iterations
    for iter = 1:maxIterations
        % Step 1: Find the closest point on each segment to each point in 'points'
        closestPoints = zeros(size(points));
        totalError = 0;
        
        for i = 1:size(points, 1)
            % Get the current point
            p = transformedPoints(i, :);
            
            % Initialize minimum distance and corresponding closest point
            minDist = inf;
            closestPoint = [0, 0];
            
            % Iterate over all segments to find the closest point on any segment
            for j = 1:length(segments)
                startPoint = segments(j).startPoint;
                endPoint = segments(j).endPoint;
                
                % Compute the closest point on the line segment
                [q, dist] = closestPointOnSegment(p, startPoint, endPoint);
                
                % Update if this is the closest point found so far
                if dist < minDist
                    minDist = dist;
                    closestPoint = q;
                end
            end
            
            % Store the closest point
            closestPoints(i, :) = closestPoint;
            totalError = totalError + minDist;
        end
        
        % Step 2: Compute the centroids of the points and closest points
        centroidPoints = mean(transformedPoints);
        centroidClosestPoints = mean(closestPoints);
        
        % Step 3: Compute the cross-covariance matrix
        H = (transformedPoints - centroidPoints)' * (closestPoints - centroidClosestPoints);
        
        % Step 4: Compute the SVD of H
        [U, ~, V] = svd(H);
        
        % Step 5: Compute the rotation matrix
        R_iter = V * U';
        
        % Step 6: Compute the translation vector
        t_iter = centroidClosestPoints' - R_iter * centroidPoints';
        
        % Step 7: Apply the transformation to the points
        transformedPoints = (R_iter * transformedPoints')' + t_iter';
        
        % Accumulate transformations
        R = R_iter * R;
        t = t_iter' + (R_iter * t')';
        
        % Step 8: Check for convergence
        if totalError < tolerance
            break;
        end
    end
end

function [closestPoint, dist] = closestPointOnSegment(p, startPoint, endPoint)
    % Compute the vector from start to end of the segment
    v = endPoint - startPoint;
    % Compute the vector from start of the segment to the point p
    w = p - startPoint;
    
    % Project point p onto the line defined by the segment
    c1 = dot(w, v);
    c2 = dot(v, v);
    
    % If c1 <= 0, closest point is startPoint
    if c1 <= 0
        closestPoint = startPoint;
        dist = norm(p - startPoint);
        return;
    end
    
    % If c1 >= c2, closest point is endPoint
    if c1 >= c2
        closestPoint = endPoint;
        dist = norm(p - endPoint);
        return;
    end
    
    % Otherwise, closest point is between startPoint and endPoint
    b = c1 / c2;
    closestPoint = startPoint + b * v;
    dist = norm(p - closestPoint);
end
