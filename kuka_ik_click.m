function kuka_ik_click()
    % Build robot
    robot = buildKukaRobot();

    % Create figure and axes
    f = figure('Name', 'KUKA IK Click Control', 'Position', [100 100 1000 600]);
    ax = axes('Parent', f);
    view(ax, 3); grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');

    % Show initial robot pose
    config = homeConfiguration(robot);
    h = show(robot, config, 'Parent', ax, 'Frames','on');
    hold on;

    % Set up inverse kinematics solver
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0.5 0.5 0.5 1 1 1]; % [xyz, rpy]
    initialGuess = config;

    % Click callback
    % Disable interactive plot tools
    zoom(f, 'off');
    pan(f, 'off');
    rotate3d(f, 'off');
    datacursormode(f, 'off');

% Now safe to set the click callback
    f.WindowButtonDownFcn = @(~,~) onClick();

    % Callback function
    function onClick()
        pt = get(gca, 'CurrentPoint');
        targetPos = pt(1,1:3);

        % Fix the orientation to point straight forward
        targetRot = eul2quat([0 0 0]); % No rotation
        tform = trvec2tform(targetPos) * quat2tform(targetRot);

        % Solve IK
        [configSol, ~] = ik('link6', tform, weights, initialGuess);
        initialGuess = configSol; % update guess for smoother motion

        % Update plot
        show(robot, configSol, 'Parent', ax, 'PreservePlot', false, 'Frames','on');
        title(ax, sprintf('Target: [%.2f, %.2f, %.2f]', targetPos));
    end
end

% --- Build KUKA Robot Function ---
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
