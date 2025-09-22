function toolpath = generateTopEdgeToolpathFromSTL(stlFile, normalZThresh, resolutionMM, shrinkFactor,offsetDistance)
    % Read STL file (returns triangulation object)
    TR = stlread(stlFile);
    V = TR.Points;
    F = TR.ConnectivityList;

    % Compute triangle normals
    triCenters = (V(F(:,1),:) + V(F(:,2),:) + V(F(:,3),:)) / 3;
    N = cross(V(F(:,2),:) - V(F(:,1),:), V(F(:,3),:) - V(F(:,1),:));
    N = N ./ vecnorm(N, 2, 2);  % normalize

    % Select top-facing triangles
    topFaceIdx = N(:,3) > normalZThresh;  % e.g. 0.9 keeps near-flat tops
    topFaces = F(topFaceIdx,:);
    topVerts = unique(topFaces(:));
    topPoints = V(topVerts, :);

    if size(topPoints, 1) < 3
        error('Not enough top surface points detected. Try lowering normalZThresh.');
    end

    % 2D boundary (XY)
    xy = topPoints(:,1:2);
    k = boundary(xy(:,1), xy(:,2), shrinkFactor);
    rawPath = topPoints(k, :);

    % Ensure closed path
    closedPath = [rawPath; rawPath(1,:)];

    % Remove consecutive duplicate points to avoid interp1 errors
    diffPts = [true; any(diff(closedPath,1,1) ~= 0, 2)];
    closedPathUnique = closedPath(diffPts, :);

    % Resample path to fixed resolution
    segLengths = vecnorm(diff(closedPathUnique), 2, 2);
    cumDist = [0; cumsum(segLengths)];
    totalLength = cumDist(end);
    queryDist = 0:resolutionMM:totalLength;
    toolpath = interp1(cumDist, closedPathUnique, queryDist, 'linear');
    initialToolpath=toolpath;
    offsetPath = offsetToolpathXY(initialToolpath, offsetDistance);

% Recompute segment lengths and cumulative distance
segLengths = vecnorm(diff(offsetPath), 2, 2);
cumDist = [0; cumsum(segLengths)];
totalLength = cumDist(end);
queryDist = 0:resolutionMM:totalLength;

% Resample offset path at fixed resolution
toolpath = interp1(cumDist, offsetPath, queryDist, 'linear');

    % Plot
    figure('Name', 'Top Edge Toolpath from STL');
    patch('Faces',F,'Vertices',V,'FaceColor',[0.8 0.8 1],'EdgeColor','none','FaceAlpha',0.3);
    hold on;

    % Plot original (unoffset) toolpath
    plot3(initialToolpath(:,1), initialToolpath(:,2), initialToolpath(:,3), 'b-', 'LineWidth', 1.5);
    
    % Plot offset toolpath
    plot3(toolpath(:,1), toolpath(:,2), toolpath(:,3), 'r-', 'LineWidth', 2);
    
    % Mark start point
    plot3(toolpath(1,1), toolpath(1,2), toolpath(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z'); title('Top Edge Toolpath');
    legend('STL Mesh','Original Toolpath','Offset Toolpath','Start Point');
    view(3); camlight; lighting gouraud;

    %% Function: XY-Only Offset (parallel to ground)
function offsetPath = offsetToolpathXY(path, offset)
    % Extract XY coordinates (discard Z for offset calculation)
    xyPath = path(:,1:2);
    
    % Compute 2D tangent vectors (direction of movement)
    tangents = diff(xyPath);
    tangents = [tangents; tangents(end,:)]; % Pad last point
    
    % Compute 2D normals (perpendicular to tangents, outward-facing)
    normals = [tangents(:,2), -tangents(:,1)]; % Rotate 90° in XY
    normals = normals ./ vecnorm(normals, 2, 2); % Normalize
    
    % Apply offset in XY (keep original Z)
    offsetPath = path;
    offsetPath(:,1:2) = xyPath + normals * offset;
end

end
