clear; close all ; clc;
file1 = readtable('C:/Users/TanMinhCAO/Desktop/Lidar0207/DataL8W4/1.csv');

% Extract X and Y coordinates
PointClouds = [file1.X, file1.Y];

toleranceInsideSegment = 0.05;
nbSegmentPtMinimum = 5;
segList = ExtractSceneSegments(PointClouds, toleranceInsideSegment, nbSegmentPtMinimum);
tailleSegmentMinimale = 0;
interestPtList = ExtractInterestPoints(segList, tailleSegmentMinimale);

figure;
hold on;
axis equal;
%plot(data2(:,1), data2(:,2), 'k-x', 'DisplayName', 'Original');
for i = 1:length(segList)
    
    x = [segList{i}.PtDebut(1), segList{i}.PtFin(1)];
    y = [segList{i}.PtDebut(2), segList{i}.PtFin(2)];
    
    plot(x, y, 'g-', 'LineWidth', 2); 
    hold on;
    plot(segList{i}.PtDebut(1), segList{i}.PtDebut(2), 'ko', 'MarkerFaceColor', 'k'); 
    plot(segList{i}.PtFin(1), segList{i}.PtFin(2), 'ko', 'MarkerFaceColor', 'k');     
end
hold on;
%plot(data2(:, 1), data2(:, 2), 'b-', 'DisplayName', 'Original');
%plot(PointClouds(:, 1), PointClouds(:, 2), 'b.', 'MarkerSize', 5, 'MarkerFaceColor', 'blue');
plot(PointClouds(:, 1), PointClouds(:, 2), 'b-', 'MarkerSize', 5, 'MarkerEdgeColor', [0.5 0.5 0.5], 'MarkerFaceColor', [0.5 0.5 0.5]);

hold on;
%plot(best_angle(:, 1), best_angle(:, 2), 'bo', 'MarkerSize', 5, 'Color', 'red');
hold on;
plot(interestPtList(:, 1), interestPtList(:, 2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'red');
hold on;
legend;
xlabel('X');
ylabel('Y');
title('');
hold off;