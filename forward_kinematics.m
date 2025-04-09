config = homeConfiguration(robot);

config(1).JointPosition = deg2rad(30);
config(2).JointPosition = deg2rad(-45);
config(3).JointPosition = deg2rad(60);
config(4).JointPosition = deg2rad(0);
config(5).JointPosition = deg2rad(45);
config(6).JointPosition = deg2rad(90);

endEffector = 'link6';
tform = getTransform(robot, config, endEffector);

position = tform2trvec(tform);
orientation = tform2eul(tform);

disp('End Effector Position (x,y,z):');
disp(position);

disp('End Effector Orientation (ZYX Euler angles in radians):');
disp(orientation);

figure;
show(robot, config, 'Frames','on', 'PreservePlot', false);
title('KUKA KR5 - Forward Kinematics');
view(3);
axis equal;