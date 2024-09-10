function approxPoints = approxPolyDP(points, epsilon)
    % approxPolyDP: Hàm đơn giản hóa các đường viền của một hình bằng thuật toán Ramer-Douglas-Peucker
    % points: Ma trận Nx2 chứa các điểm [x, y]
    % epsilon: Độ chính xác đơn giản hóa
    
    % Nếu có ít hơn 3 điểm, trả về các điểm như hiện tại
    if size(points, 1) < 3
        approxPoints = points;
        return;
    end
    
    % Tìm điểm có khoảng cách lớn nhất
    dmax = 0;
    index = 0;
    for i = 2:(size(points, 1) - 1)
        d = pointLineDist(points(i, :), points(1, :), points(end, :));
        if d > dmax
            index = i;
            dmax = d;
        end
    end
    
    % Nếu khoảng cách lớn nhất lớn hơn epsilon, đơn giản hóa đệ quy
    if dmax > epsilon
        % Gọi đệ quy
        recResults1 = approxPolyDP(points(1:index, :), epsilon);
        recResults2 = approxPolyDP(points(index:end, :), epsilon);
        
        % Xây dựng danh sách kết quả
        approxPoints = [recResults1(1:end-1, :); recResults2];
    else
        % Nếu không, trả về hai điểm đầu mút
        approxPoints = [points(1, :); points(end, :)];
    end
end

