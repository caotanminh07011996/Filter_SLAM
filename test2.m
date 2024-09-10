clear; close all ; clc;
file1 = readtable('C:/Users/TanMinhCAO/Desktop/Lidar0207/DataL8W4/1.csv');

% Extract X and Y coordinates
polarLidarPoints = [file1.X, file1.Y];
toleranceInsideSegment = 0.05;
nbSegmentPtMinimum = 5;


vertexList = approxPolyDP(polarLidarPoints, toleranceInsideSegment);
    
j = 0;
endOfSegmentIndex = [];
for i = 1:size(vertexList, 1)
    while ~isequal(vertexList(i, :), polarLidarPoints(j+1, :))
        j = j + 1;
        if j >= size(polarLidarPoints, 1)
        j = 1;
        end
    end
    endOfSegmentIndex = [endOfSegmentIndex; j+1];
end

segList = {};
for i = 1:size(vertexList, 1)
    if i > 1
        indexPtLidarDebutSegment = endOfSegmentIndex(i - 1);
        vertexDebutSegment = vertexList(i - 1, :);
    else
        indexPtLidarDebutSegment = endOfSegmentIndex(end);
        vertexDebutSegment = vertexList(end, :);
    end

    indexPtLidarFinSegment = endOfSegmentIndex(i);
    vertexFinSegment = vertexList(i, :);

    ptDebut = vertexDebutSegment;
    ptFin = vertexFinSegment;

    ptDebut_X = vertexDebutSegment(1);
    ptDebut_Y = vertexDebutSegment(2);
    ptFin_X = vertexFinSegment(1);
    ptFin_Y = vertexFinSegment(2);

    angleSegment = Modulo2Pi(-Angle([0, 0], ptFin)+Angle(ptDebut, ptFin));

    s.PtDebut = ptDebut;
    s.PtFin = ptFin;
    s.indexPtLidarDebutSegment = indexPtLidarDebutSegment;
    s.indexPtLidarFinSegment = indexPtLidarFinSegment;
    s.angleSegmentRefLidar = angleSegment;

    segList{end + 1} = s;
end

filteredSegList = {};
for i = 1:length(segList)
    s = segList{i};
    if abs(ModuloPi(s.angleSegmentRefLidar)) >= deg2rad(20)
            
        if s.indexPtLidarFinSegment - s.indexPtLidarDebutSegment >= 0
            seg.PtDebut = s.PtDebut;
            seg.PtFin = s.PtFin;
            seg.indexPtLidarDebutSegment = s.indexPtLidarDebutSegment;
            seg.indexPtLidarFinSegment = s.indexPtLidarFinSegment;
            seg.angleSegmentRefLidar = s.angleSegmentRefLidar;
                    
            if i ~= 1
                seg.angleSegmentRefLidarPrecedent = segList{i - 1}.angleSegmentRefLidar;
            else
                seg.angleSegmentRefLidarPrecedent = segList{end}.angleSegmentRefLidar;
            end

            if i < length(segList)
                seg.angleSegmentRefLidarSuivant = segList{i + 1}.angleSegmentRefLidar;
            else
                seg.angleSegmentRefLidarSuivant = segList{1}.angleSegmentRefLidar;
            end
            filteredSegList{end + 1} = seg;
         end
          
    end
end