
function po=tersst(innerdia)
% Fetch image from URL
url = 'http://192.168.56.101:4242/current.jpg?type=color';
img = webread(url);
gray = rgb2gray(img);
bo=gray;
%bo = im2double(gray);

% Set your desired black/white points 
% (adjust these values based on your image)
black_point = 0.3;  % Pixels below this will become pure black (0)
white_point = 0.8;  % Pixels above this will become pure white (1)

% Apply contrast stretching
gray = imadjust(bo, [black_point; white_point], [0; 1]);


% Define ROI (Region of Interest) - Middle and Left Part
[height, width] = size(gray);
roi_x_start = round(width * 0.3);    % Start ~30% from left
roi_x_end   = round(width * 0.7);    % End ~60% from left
roi_y_start = round(height * 0.3);   % Start ~30% from top
roi_y_end   = round(height * 0.8);   % End ~60% from top

% Crop the ROI
gray_roi = gray(roi_y_start:roi_y_end, roi_x_start:roi_x_end);

% Preprocessing
gray_roi = imadjust(gray_roi);            % Improve contrast
gray_roi = imgaussfilt(gray_roi, 1.5);   % Mild smoothing

% Search for the most robust dark circle with adaptive sensitivity
best_center = [];
best_radius = [];
min_sensitivity = 0.5;
max_sensitivity = 0.99;
step = 0.02;

for sensitivity = min_sensitivity:step:max_sensitivity
   for polarity = ["dark"]
    [centers, radii] = imfindcircles(gray_roi, [6 80], ...
        'ObjectPolarity', polarity, ...
        'Sensitivity', sensitivity);
   end
    if ~isempty(centers)
        best_center = centers(1, :);
        best_radius = radii(1);

        % Adjust coordinates to full image frame
        best_center(1) = best_center(1) + roi_x_start - 1;
        best_center(2) = best_center(2) + roi_y_start - 1;
        break;  % Stop at first successful detection
    end
end

% Check detection success
if isempty(best_center)
    error('No circle detected in the specified ROI.');
end

% Image size in pixels
image_width_px = size(img, 2)
image_height_px = size(img, 1)

% Camera FOV in degrees
fov_h_deg = 50;
fov_v_deg = 39;

% Convert FOV to radians
fov_h = deg2rad(fov_h_deg);
fov_v = deg2rad(fov_v_deg);

% Calculate focal lengths in pixels
f_x = (image_width_px / 2) / tan(fov_h / 2);
f_y = (image_height_px / 2) / tan(fov_v / 2);
% Known real diameter of the object (in mm)
real_diameter_mm = innerdia;

% Circle diameter in pixels (from detection)
diameter_px = 2 * best_radius;

% Estimate depth Z (in mm)
Z_mm = (f_x * real_diameter_mm) / diameter_px;

g=real_diameter_mm/diameter_px;
% Image center (optical center)
c_x = image_width_px / 2
c_y = image_height_px / 2

% Pixel coordinates of circle center
u = best_center(1);
v = best_center(2);
best_center=best_center;
% Convert pixel coordinates to real world X, Y (in mm)
X_mm = ((u - c_x)*g); %* Z_mm) / f_x;
Y_mm = ((v - c_y)*g); %* Z_mm) / f_y;

% Final 3D position of object relative to camera frame (mm)
position_mm_camera = [X_mm; Y_mm; Z_mm];

fprintf('Estimated 3D position of the object in camera frame (mm):\n');
fprintf('X = %.2f mm, Y = %.2f mm, Z = %.2f mm\n', X_mm, Y_mm, Z_mm);
best_radius=best_radius

% --- Part 2: ROS 2 - get robot joint states and compute end-effector pose ---

%Initialize ROS2 node and subscriber (adjust if node already exists)
node = ros2node("/matlab_node");
jointSub = ros2subscriber(node, "/joint_states", "sensor_msgs/JointState");

% Receive joint states (wait max 5 seconds)
jointMsg = receive(jointSub, 5);

% UR3e joint names in order
jointOrder = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', ...
   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

jointAngles = zeros(1,6);
for i = 1:6
   idx = find(strcmp(jointMsg.name, jointOrder{i}));
   if isempty(idx)
       error("Joint %s not found in joint state message", jointOrder{i});
   end
   jointAngles(i) = jointMsg.position(idx);
end

% Use your forward kinematics function here (copy this function below or keep in path)
[position_ee, orientation_ee] = ur3e_forward_kinematics(jointAngles);
camera=zeros(1,3);

camera(1,2)=37.5*cosd(30)+2;
camera(1,3)=-37.5*sind(30);
position_mm_camera(1)=position_mm_camera(1)+11;
position_mm_camera(2)=-position_mm_camera(2);
position_mm_camera(3)=-position_mm_camera(3);
for n=1:3
    po(1,n)= position_ee(n)*1000+position_mm_camera(n)+camera(n);
end
% Build transform matrix from end-effector pose
R_arm = quat2rotm(orientation_ee);
T_arm = [R_arm, position_ee'; 0 0 0 1];

% --- Part 3: Camera-to-end-effector transform ---

% If camera is mounted at end-effector with a known fixed offset:
theta = deg2rad(30);  % example rotation 30 deg around X axis
R_x = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
t = [0; 0; 0]; % 37.5 mm offset in Y (convert mm to meters)
T_camera_to_ee = [R_x, t; 0 0 0 1];

% If no offset, just use identity
% T_camera_to_ee = eye(4);

% Convert object position from mm to meters
p_camera_m = position_mm_camera / 1000;
p_camera_hom = [p_camera_m; 1];

% Compute object position in robot base frame
T_camera_in_base = T_arm * T_camera_to_ee;
p_object_base_hom = T_camera_in_base + p_camera_hom;
p_object_base = p_object_base_hom(1:3);
%po= p_object_base;

fprintf('End-effector position (m): X=%.3f Y=%.3f Z=%.3f\n', position_ee);
fprintf('Object position relative to robot base (m): X=%.3f Y=%.3f Z=%.3f\n', ...
    p_object_base(1), p_object_base(2), p_object_base(3));

% --- Visualization ---

figure; imshow(gray); hold on;
viscircles(best_center, best_radius, 'EdgeColor', 'r');
plot(best_center(1), best_center(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
title('Detected Circle on Camera Image');
figure; imshow(gray_roi); hold on;
viscircles(centers, radii, 'EdgeColor', 'b');
title('Circle detection in ROI');

end
function [position, orientation] = ur3e_forward_kinematics(jointAngles)
%#codegen

% Inputs:
% jointAngles - 1x6 array [rad] of UR3e joint angles
% Outputs:
% position - 1x3 [m] XYZ position of tool tip
% orientation - 1x4 quaternion [qx qy qz qw] of tool tip

% Load robot model once and persist in memory
persistent robot toolTransform
if isempty(robot)
    robot = loadrobot('universalUR3e', 'DataFormat', 'row');
    
    % Define tool transform (211 mm link + 4.5 mm tool radius)
    toolOffset = trvec2tform([0 0 0]);  % in meters
    toolRotation = axang2tform([1 0 0 pi]);    % flip around X
    toolTransform = toolRotation * toolOffset;
end

% Compute forward kinematics to 'tool0'
tform = getTransform(robot, jointAngles, 'tool0');

% Apply tool transform
fullTform = tform * toolTransform;

% Extract position and orientation
position = fullTform(1:3, 4)';
rotm = fullTform(1:3, 1:3);

orientation = rotm2quat(rotm);  % [qx qy qz qw]
end
