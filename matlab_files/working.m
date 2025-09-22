%% 1. Load robot (unchanged)
robot = loadrobot('universalUR3e', 'DataFormat', 'row');
jointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint', ...
              'wrist_1_joint','wrist_2_joint','wrist_3_joint'};

%% 2. Load STL and create XY-offset toolpath
%ptCloud = stlToPointCloud('disk.stl', 10000);  % High-res for smooth offset
offsetDistance = 3.18; % mm (horizontal offset)
innerdia=16;
outerdia=18;

createPipeSTL(innerdia, outerdia, 'disk.stl');
% Generate initial top-edge toolpath (Z varies, XY follows shape)
%initialToolpath = generateTopEdgeToolpathFromPC(ptCloud, 0.01, 0.7, 11);
initialToolpath = generateTopEdgeToolpathFromSTL('disk.stl', 0.9, 0.5, 0.3,offsetDistance);

% Compute XY-parallel offset (Z remains unchanged)
%toolpath = offsetToolpathXY(initialToolpath, offsetDistance);
toolpath    = initialToolpath;
% Apply global offset (optional)
po=tersst(innerdia)
po(3)=zppo*1000-0.5;
p=po
toolpath(:,1) = toolpath(:,1) + po(1);
toolpath(:,3) = toolpath(:,3) + po(3);
toolpath(:,2)=  toolpath(:,2) + po(2);


%% Rest of your code (IK, visualization, etc.) remains the same...
%% 3. Define tool transform: 211 mm + 4.5 mm ball radius
toolOffset = trvec2tform([0 0 -0.29418]);  % in meters
toolRotation = axang2tform([1 0 0 pi]);  % flip 180° around X (downward)
toolTransform = toolRotation * toolOffset;

%% 4. Inverse Kinematics
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.1, 0.1, 0.1, 1, 1, 1];
initialGuess = [0 -pi/2 pi/2 -pi/2 -pi/2 0];
jointAngles = zeros(size(toolpath,1), 6);
validPoses = true(size(toolpath,1), 1);

for i = 1:size(toolpath,1)
    pos = toolpath(i,1:3) / 1000;  % mm to meters

    % Tool should point downward: fixed orientation
    orientation = eul2rotm([0 0 0], 'XYZ');  % face down
    pose = [orientation, pos'; 0 0 0 1];
    fullPose = pose * toolTransform;

    [config, info] = ik('tool0', fullPose, weights, initialGuess);
    if ~strcmp(info.Status, 'success')
        jointAngles(i,:) = NaN;
        validPoses(i) = false;
    else
        jointAngles(i,:) = config;
        initialGuess = config;
    end
end


% Filter out invalid poses
validAngles = jointAngles(validPoses, :);
toolpath = toolpath(validPoses, :);

%% 5. Save joint poses to .mat file
save('ur10e_joint_poses.mat', 'validAngles', 'toolpath');
disp('✅ Saved joint poses to ur10e_joint_poses.mat');

%% 6. Plot 3D Toolpath
% figure;
% plot3(toolpath(:,1), toolpath(:,2), toolpath(:,3), 'b.-');
% grid on; axis equal;
% xlabel('X (mm)');
% ylabel('Y (mm)');
% zlabel('Z (mm)');
% title('Generated Toolpath in 3D');

%% STL Transformation Tool - Direct Script Version

% ===== USER CONFIGURATION =====
inputFile = 'disk.stl';         % Input STL filename
outputFile = 'diskposition.stl'; % Output STL filename

% Set your desired transformations:
translation = [po(1), po(2), -po(3)];     % [X, Y, Z] translation in mm
rotation = [0, 0, 0];          % [X-rot, Y-rot, Z-rot] in degrees
% ==============================

%% Load STL file
try
    % Try MATLAB's built-in stlread first
    model = stlread(inputFile);
    vertices = model.Points;
    faces = model.ConnectivityList;
    disp('Loaded STL using built-in reader');
catch
    % Fallback to custom reader
    disp('Using custom STL reader');
    [vertices, faces] = customSTLReader(inputFile);
end

%% Display original center point
original_center = mean(vertices);
fprintf('\nOriginal STL center: [%.2f, %.2f, %.2f] mm\n', original_center);

%% Apply transformations
% Create transformation matrix
tform = makehgtform('translate', translation);

% Apply rotations (convert degrees to radians)
if rotation(1) ~= 0
    tform = tform * makehgtform('xrotate', deg2rad(rotation(1)));
end
if rotation(2) ~= 0
    tform = tform * makehgtform('yrotate', deg2rad(rotation(2)));
end
if rotation(3) ~= 0
    tform = tform * makehgtform('zrotate', deg2rad(rotation(3)));
end

% Apply to all vertices
homogenous_coords = [vertices, ones(size(vertices,1),1)];
transformed_vertices = (tform * homogenous_coords')';
transformed_vertices = transformed_vertices(:,1:3);

%% Verify transformation
new_center = mean(transformed_vertices);
fprintf('New STL center: [%.2f, %.2f, %.2f] mm\n', new_center);

% Check translation
tolerance = 0.01; % 0.01 mm tolerance
translation_error = (new_center - original_center) - translation;

if all(abs(translation_error) < tolerance)
    fprintf('✓ Translation verified within tolerance\n');
else
    warning('Translation error: [%.2f, %.2f, %.2f] mm', translation_error);
end

%% Save transformed STL
if exist('stlwrite', 'file')
    stlwrite(triangulation(faces, transformed_vertices), outputFile);
    disp('Saved using built-in stlwrite');
else
    customSTLWriter(outputFile, faces, transformed_vertices);
    disp('Saved using custom writer');
end

%% Visualize results
figure;
subplot(1,2,1);
trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3));
title('Original STL'); axis equal; grid on;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

subplot(1,2,2);
trisurf(faces, transformed_vertices(:,1), transformed_vertices(:,2), transformed_vertices(:,3));
title('Transformed STL'); axis equal; grid on;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

%% Custom STL Reader/Writer Functions
function [vertices, faces] = customSTLReader(filename)
    fid = fopen(filename, 'r');
    header = fread(fid, 80, 'uchar=>char')';
    is_ascii = contains(header, 'solid');
    fclose(fid);
    
    if is_ascii
        data = fileread(filename);
        v = regexp(data, 'vertex\s+([\d\.\-+eE]+)\s+([\d\.\-+eE]+)\s+([\d\.\-+eE]+)', 'tokens');
        vertices = str2double(vertcat(v{:}));
        faces = reshape(1:size(vertices,1), 3, [])';
    else
        fid = fopen(filename, 'rb');
        fread(fid, 80, 'uchar=>char');
        nf = fread(fid, 1, 'uint32');
        vertices = zeros(nf*3, 3);
        for i = 1:nf
            fread(fid, 3, 'float32');
            vertices((i-1)*3+1:i*3,:) = fread(fid, [3,3], 'float32')';
            fread(fid, 1, 'uint16');
        end
        fclose(fid);
        faces = reshape(1:nf*3, 3, [])';
    end
end

function customSTLWriter(filename, faces, vertices)
    fid = fopen(filename, 'w');
    fprintf(fid, 'solid %s\n', filename);
    for i = 1:size(faces,1)
        v1 = vertices(faces(i,1),:);
        v2 = vertices(faces(i,2),:);
        v3 = vertices(faces(i,3),:);
        n = cross(v2-v1, v3-v1);
        n = n/norm(n);
        
        fprintf(fid, 'facet normal %.7g %.7g %.7g\n', n);
        fprintf(fid, ' outer loop\n');
        fprintf(fid, '  vertex %.7g %.7g %.7g\n', v1);
        fprintf(fid, '  vertex %.7g %.7g %.7g\n', v2);
        fprintf(fid, '  vertex %.7g %.7g %.7g\n', v3);
        fprintf(fid, ' endloop\n');
        fprintf(fid, 'endfacet\n');
    end
    fprintf(fid, 'endsolid %s\n', filename);
    fclose(fid);
end

% Filter out invalid poses
validAngles = jointAngles(validPoses, :);
toolpath = toolpath(validPoses, :);

% === NEW: Compute toolpath length and number of points ===
diffs = diff(toolpath(:,1:3), 1, 1);
segment_lengths = sqrt(sum(diffs.^2, 2));
toolpath_length = sum(segment_lengths); % in mm
toolpath_num_points = size(toolpath, 1);

% Save for Simulink
save('toolpath_info.mat', 'toolpath_length', 'toolpath_num_points');
disp(['✅ Toolpath length: ', num2str(toolpath_length, '%.2f'), ' mm, Points: ', num2str(toolpath_num_points)]);
disp('✅ Saved toolpath info to toolpath_info.mat');
