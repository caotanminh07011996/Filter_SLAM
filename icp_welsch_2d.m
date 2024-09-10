function [R, t] = icp_welsch_2d(source, target, max_iter, tol, c)
    % ICP with Welsch loss function for 2D points
    % Inputs:
    %   - source: Nx2 matrix of source points
    %   - target: Nx2 matrix of target points
    %   - max_iter: maximum number of iterations
    %   - tol: tolerance for convergence
    %   - c: tuning constant for Welsch loss function
    % Outputs:
    %   - R: rotation matrix
    %   - t: translation vector

    % Initialize R and t
    R = eye(2);
    t = zeros(2, 1);
    prev_error = inf;

    for iter = 1:max_iter
        % Transform source points using current R and t
        transformed_source = (R * source' + t)';
        
        %Step1: Find the closest points in the target set
        [indices, distances] = knnsearch(target, transformed_source);
        
        % Compute Welsch weights
        w = exp(- (distances / c).^2);
        
        % Weighted centroids
        W = diag(w);
        source_weighted_centroid = sum(W * source) / sum(w);
        target_weighted_centroid = sum(W * target(indices, :)) / sum(w);
        
        % Center the points
        source_centered = source - source_weighted_centroid;
        target_centered = target(indices, :) - target_weighted_centroid;
        
        % Compute the covariance matrix
        H = source_centered' * W * target_centered;
        
        % Singular Value Decomposition (SVD)
        [U, ~, V] = svd(H);
        
        % Compute rotation
        R = V * U';
        
        % Ensure a right-handed coordinate system
        if det(R) < 0
            V(:, end) = -V(:, end);
            R = V * U';
        end
        
        % Compute translation
        t = target_weighted_centroid' - R * source_weighted_centroid';
        
        % Apply the transformation to the source points
        transformed_source = (R * source' + t)';
        
        % Compute the current error using Welsch loss
        error = sum(w .* sqrt(sum((transformed_source - target(indices, :)).^2, 2)));
        
        % Check for convergence
        if abs(prev_error - error) < tol
            break;
        end
        
        prev_error = error;
    end
end
