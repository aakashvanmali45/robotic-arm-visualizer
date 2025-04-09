% Build robot
robot = buildKukaRobot(); % same function used before

% Define the inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);

% Define the desired end-effector pose (position + orientation)
targetPosition = [0.6, 0.2, 0.7];  % x, y, z in meters
targetOrientation = eul2quat([pi/2, 0, pi/4]); % [yaw, pitch, roll] → quaternion

% Build transformation matrix
tform = trvec2tform(targetPosition) * quat2tform(targetOrientation);

% Weights for position [x y z] and orientation [roll pitch yaw]
weights = [0.5, 0.5, 0.5, 1, 1, 1]; % prioritize orientation more

% Use homeConfiguration as the initial guess
initialGuess = homeConfiguration(robot);

% Solve IK
[configSol, solInfo] = ik('link6', tform, weights, initialGuess);

% Show results
disp('IK Solution (Joint Angles in Degrees):');
for i = 1:6
    fprintf('Joint %d: %.2f°\n', i, rad2deg(configSol(i).JointPosition));
end

% Visualize
figure;
show(robot, configSol, 'Frames','on');
title('Inverse Kinematics Solution');
axis equal;


function robot = buildKukaRobot()
    robot = rigidBodyTree('DataFormat','struct','MaxNumBodies',6);

    % DH Parameters [a d alpha]
    dh = [ 0     0.4   -pi/2;
           0.25  0     0;
           0.68  0    -pi/2;
           0     0.67  pi/2;
           0     0    -pi/2;
           0     0.158 0];

    prevBody = 'base';
    for i = 1:6
        body = rigidBody(['link' num2str(i)]);
        joint = rigidBodyJoint(['joint' num2str(i)], 'revolute');

        a = dh(i,1); d = dh(i,2); alpha = dh(i,3);
        T = trvec2tform([a, 0, d]) * axang2tform([1 0 0 alpha]);
        setFixedTransform(joint, T);

        body.Joint = joint;
        addBody(robot, body, prevBody);
        prevBody = body.Name;
    end
end
