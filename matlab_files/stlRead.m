function [v, f, n] = stlRead(filename)
    % Read ASCII STL file
    fid = fopen(filename, 'r');
    if fid == -1
        error('File could not be opened');
    end
    
    % Read all data
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    data = data{1};
    
    % Find vertices
    vLines = contains(data, 'vertex');
    vertices = data(vLines);
    v = zeros(length(vertices), 3);
    for i = 1:length(vertices)
        v(i,:) = sscanf(vertices{i}, ' vertex %f %f %f');
    end
    
    % Find faces (each face has 3 vertices)
    f = reshape(1:length(v), 3, [])';
    
    % Calculate normals if needed
    if nargout > 2
        n = zeros(size(f,1), 3);
        for i = 1:size(f,1)
            v1 = v(f(i,2),:) - v(f(i,1),:);
            v2 = v(f(i,3),:) - v(f(i,1),:);
            n(i,:) = cross(v1, v2);
            n(i,:) = n(i,:)/norm(n(i,:));
        end
    end
end

function stlWrite(filename, faces, vertices)
    % Write ASCII STL file
    fid = fopen(filename, 'w');
    if fid == -1
        error('File could not be opened for writing');
    end
    
    fprintf(fid, 'solid %s\n', filename);
    
    for i = 1:size(faces,1)
        % Calculate normal
        v1 = vertices(faces(i,2),:) - vertices(faces(i,1),:);
        v2 = vertices(faces(i,3),:) - vertices(faces(i,1),:);
        n = cross(v1, v2);
        n = n/norm(n);
        
        fprintf(fid, 'facet normal %f %f %f\n', n);
        fprintf(fid, ' outer loop\n');
        fprintf(fid, '  vertex %f %f %f\n', vertices(faces(i,1),:));
        fprintf(fid, '  vertex %f %f %f\n', vertices(faces(i,2),:));
        fprintf(fid, '  vertex %f %f %f\n', vertices(faces(i,3),:));
        fprintf(fid, ' endloop\n');
        fprintf(fid, 'endfacet\n');
    end
    
    fprintf(fid, 'endsolid %s\n', filename);
    fclose(fid);
end