function [R, T, transformed_source] = ICP(source, target, max_iterations, tolerance)
    % ICP_LIDAR_2D Performs the Iterative Closest Point algorithm for 2D point clouds.
    %
    % Inputs:
    %   source - Nx2 matrix of source point cloud (each row is a point [x, y])
    %   target - Mx2 matrix of target point cloud (each row is a point [x, y])
    %   max_iterations - Maximum number of iterations
    %   tolerance - Convergence tolerance
    %
    % Outputs:
    %   R - Rotation matrix
    %   T - Translation vector
    %   transformed_source - Transformed source point cloud

    % Initialize transformation
    R = eye(2);
    T = [0; 0];
    transformed_source = source;

    prev_error = inf;

    for iter = 1:max_iterations
        % Step 1: Find closest points
        [indices, dists] = knnsearch(target, transformed_source);
        closest_points = target(indices, :);

        % Step 2: Compute centroids
        centroid_source = mean(transformed_source, 1);
        centroid_target = mean(closest_points, 1);

        % Step 3: Compute centered vectors
        source_centered = transformed_source - centroid_source;
        target_centered = closest_points - centroid_target;

        % Step 4: Compute covariance matrix
        H = source_centered' * target_centered;

        % Step 5: Compute SVD
        [U, ~, V] = svd(H);
        R_new = V * U';
        if det(R_new) < 0
            V(:, end) = -V(:, end);
            R_new = V * U';
        end

        % Step 6: Compute translation
        T_new = centroid_target' - R_new * centroid_source';

        % Step 7: Update transformation
        R = R_new * R;
        T = R_new * T + T_new;

        % Apply transformation
        transformed_source = (R_new * source' + T_new)';

        % Compute mean squared error
        error = mean(dists);

        % Check for convergence
        if abs(prev_error - error) < tolerance
            break;
        end
        prev_error = error;
    end
end