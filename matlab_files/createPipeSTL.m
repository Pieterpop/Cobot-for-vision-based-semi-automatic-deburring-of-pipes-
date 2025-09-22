function createPipeSTL(innerDiameter, outerDiameter, filename)
    % CREATE PIPESTL Generates a pipe STL model
    %   innerDiameter: inner diameter of the pipe (mm)
    %   outerDiameter: outer diameter of the pipe (mm)
    %   filename: output STL filename (e.g., 'pipe.stl')
    
    % Validate inputs
    if outerDiameter <= innerDiameter
        error('Outer diameter must be larger than inner diameter');
    end
    
    % Parameters
    length = 50; % 5 cm = 50 mm (in negative z-direction)
    numSides = 500; % Number of sides for the circular cross-section
    
    % Create vertices for outer and inner cylinders
    theta = linspace(0, 2*pi, numSides)';
    theta(end) = []; % Remove duplicate last point (2pi = 0)
    numSides = numSides - 1;
    
    % Outer cylinder vertices (top and bottom)
    outerTop = [outerDiameter/2 * cos(theta), outerDiameter/2 * sin(theta), zeros(numSides, 1)];
    outerBottom = [outerDiameter/2 * cos(theta), outerDiameter/2 * sin(theta), -length*ones(numSides, 1)];
    
    % Inner cylinder vertices (top and bottom)
    innerTop = [innerDiameter/2 * cos(theta), innerDiameter/2 * sin(theta), zeros(numSides, 1)];
    innerBottom = [innerDiameter/2 * cos(theta), innerDiameter/2 * sin(theta), -length*ones(numSides, 1)];
    
    % Combine all vertices
    vertices = [outerTop; outerBottom; innerTop; innerBottom];
    
    % Create faces for outer cylinder
    facesOuter = [];
    for i = 1:numSides-1
        facesOuter = [facesOuter; 
                     i, i+1, i+1+numSides;
                     i, i+1+numSides, i+numSides];
    end
    % Connect last segment
    facesOuter = [facesOuter;
                 numSides, 1, 1+numSides;
                 numSides, 1+numSides, numSides+numSides];
    
    % Create faces for inner cylinder (normals point inward)
    innerOffset = 2*numSides;
    facesInner = [];
    for i = 1:numSides-1
        facesInner = [facesInner; 
                     innerOffset+i, innerOffset+i+numSides, innerOffset+i+1;
                     innerOffset+i+1, innerOffset+i+numSides, innerOffset+i+1+numSides];
    end
    % Connect last segment
    facesInner = [facesInner;
                 innerOffset+numSides, innerOffset+2*numSides, innerOffset+1;
                 innerOffset+1, innerOffset+2*numSides, innerOffset+1+numSides];
    
    % Create top and bottom faces (annular rings)
    facesTop = [];
    facesBottom = [];
    for i = 1:numSides-1
        % Top face (outer then inner)
        facesTop = [facesTop;
                   i, i+1, innerOffset+i+1;
                   i, innerOffset+i+1, innerOffset+i];
        % Bottom face (outer then inner)
        facesBottom = [facesBottom;
                     i+numSides, innerOffset+i+1+numSides, i+1+numSides;
                     i+numSides, innerOffset+i+numSides, innerOffset+i+1+numSides];
    end
    % Connect last segment for top and bottom
    facesTop = [facesTop;
               numSides, 1, innerOffset+1;
               numSides, innerOffset+1, innerOffset+numSides];
    facesBottom = [facesBottom;
                 2*numSides, innerOffset+1+numSides, numSides+1;
                 2*numSides, innerOffset+numSides, innerOffset+1+numSides];
    
    % Combine all faces
    faces = [facesOuter; facesInner; facesTop; facesBottom];
    
    % Create triangulation and STL file
    pipeModel = triangulation(faces, vertices);
    
    % Write to STL file
    stlwrite(pipeModel, filename);
    
    disp(['Pipe STL file saved as: ', filename]);
end