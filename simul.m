a=experience.Observation.CarState.Data(:,:,1);

path = [2.00    1.00;
        3.00    3.00;
        4.00    4.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        10.00   10.00;
        11.75   10.75;
        12.00   10.00];

robotInitialLocation=[a(1),a(2)];

robotGoal = path(end,:);

initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
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
while i<119
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
    xlim([0 13])
    ylim([0 13])
    reward=reward+exp(-1*(e_err));
    i=i+1;
    waitfor(vizRate);
end



function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = norm(path(i,:) - current_position);
    end
    [e_err,index] = min(distances);
end