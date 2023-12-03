a=experience.Observation.CarState.Data(:,:,1);

path = [-20.00   -20.00;        -19.50   -19.01;        -19.00   -18.05;        -18.50   -17.11;        -18.00   -16.20;        -17.50   -15.31;        -17.00   -14.45;        -16.50   -13.61;        -16.00   -12.80;        -15.50   -12.01;        -15.00   -11.25;        -14.50   -10.51;        -14.00    -9.80;        -13.50    -9.11;        -13.00    -8.45;        -12.50    -7.81;        -12.00    -7.20;        -11.50    -6.61;        -11.00    -6.05;        -10.50    -5.51;        -10.00    -5.00;        -9.50     -4.51;        -9.00     -4.05;        -8.50     -3.61;        -8.00     -3.20;        -7.50     -2.81;        -7.00     -2.45;        -6.50     -2.11;        -6.00     -1.80;        -5.50     -1.51;        -5.00     -1.25;        -4.50     -1.01;        -4.00     -0.80;        -3.50     -0.61;        -3.00     -0.45;        -2.50     -0.31;        -2.00     -0.20;        -1.50     -0.11;        -1.00     -0.05;        -0.50     -0.01;         0.00      0.00;         0.50     -0.01;         1.00     -0.05;         1.50     -0.11;         2.00     -0.20;         2.50     -0.31;         3.00     -0.45;         3.50     -0.61;         4.00     -0.80;         4.50     -1.01;         5.00     -1.25;         5.50     -1.51;         6.00     -1.80;         6.50     -2.11;         7.00     -2.45;         7.50     -2.81;         8.00     -3.20;         8.50     -3.61;         9.00     -4.05;         9.50     -4.51;         10.00    -5.00;         10.50    -5.51;         11.00    -6.05;         11.50    -6.61;         12.00    -7.20;         12.50    -7.81;         13.00    -8.45;         13.50    -9.11;         14.00    -9.80;         14.50   -10.51;         15.00   -11.25;         15.50   -12.01;         16.00   -12.80;         16.50   -13.61;         17.00   -14.45;         17.50   -15.31;         18.00   -16.20;         18.50   -17.11;         19.00   -18.05;         19.50   -19.01;         20.00   -20.00];

robotInitialLocation=[a(1),a(2)];

robotGoal = path(end,:);

initialOrientation = pi/2;

robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

figure
plot(path(:,1), path(:,2),'k--d')
xlim([-20 20])
ylim([-20 0])

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 3;
controller.MaxAngularVelocity = 2;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

reward=0;

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;
i=1;
iter=size(experience.Observation.CarState.Time);
iter=iter(1);
while i<iter
    b=experience.Action.CarAction.Data(i);
    controller.LookaheadDistance = b;

    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    [e_err,~]=calculate_e(path, robotCurrentPose(1:2));
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([-20 20])
    ylim([-20 0])
    reward=reward+exp(-1*(e_err));
    i=i+1;
    waitfor(vizRate);
end

hold off
yy=experience.Action.CarAction.Data(:);
plot(1:iter-1,yy)

function [e_err, index,distances]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = sqrt((path(i,1)-current_position(1))^2+(path(i,2)-current_position(2))^2);
    end
    [e_err,index] = min(distances);
end