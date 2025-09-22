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

% Calculate forward kinematics with 296 mm tool length
[z_position] = ur3e_z_position_with_tool(jointAngles);

fprintf('Tool tip Z position (m): %.3f\n', z_position);

function [z_position] = ur3e_z_position_with_tool(jointAngles)
%#codegen

% Inputs:
% jointAngles - 1x6 array [rad] of UR3e joint angles
% Outputs:
% z_position - Z position of tool tip (296 mm from flange)

% Load robot model once and persist in memory
persistent robot toolTransform
if isempty(robot)
    robot = loadrobot('universalUR3e', 'DataFormat', 'row');
    
    % Define tool transform (296 mm tool length)
    toolOffset = trvec2tform([0 0 0.296]);  % 296 mm in Z direction
    toolTransform = toolOffset;
end

% Compute forward kinematics to 'tool0'
tform = getTransform(robot, jointAngles, 'tool0');

% Apply tool transform
fullTform = tform * toolTransform;

% Extract just the Z position
z_position = fullTform(3, 4);
end
zppo=z_position;