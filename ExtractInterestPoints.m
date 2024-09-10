function interestPtList = ExtractInterestPoints(baseSegmentList, tailleSegmentMinimale)

    
    if tailleSegmentMinimale == 0
        
        longueurMoyenne = mean(cellfun(@(s) norm([s.PtFin(1)- s.PtDebut(1), s.PtFin(2) - s.PtDebut(2)]), baseSegmentList));
        tailleSegmentMinimale = longueurMoyenne;
    else
        
        tailleSegmentMinimale = tailleSegmentMinimale ;
    end
    
    
    interestPtList = [];
    
    % Loop through each segment in the cell array baseSegmentList
    for i = 1:length(baseSegmentList)
        segment = baseSegmentList{i};  % Access the structure in the current cell
        
        % Calculate the norm (length) of the current segment using PtDebut and PtFin
        segmentNorm = norm([segment.PtFin(1)- segment.PtDebut(1), segment.PtFin(2) - segment.PtDebut(2)]);
        
        % Only consider segments longer than the minimum size
        if segmentNorm > tailleSegmentMinimale
            % Check angle for segment's end point
            if isnan(segment.angleSegmentRefLidar) || ...
               abs(modulo2PiAroundAngle(pi, segment.angleSegmentRefLidarSuivant) - pi) < degtorad(5)
                % Do nothing if angle condition is satisfied
            else
                % Add the segment's end point to interestPtList
                interestPtList = [interestPtList; [segment.PtFin(1), segment.PtFin(2)]];
            end
            
            % Check angle for segment's start point
            if isnan(segment.angleSegmentRefLidar) || ...
               abs(modulo2PiAroundAngle(0, segment.angleSegmentRefLidarPrecedent) - 0) < degtorad(5)
                % Do nothing if angle condition is satisfied
            else
                % Add the segment's start point to interestPtList
                interestPtList = [interestPtList; [segment.PtDebut(1), segment.PtDebut(2)]];
            end
        end
    end
end
