function [NextObs,Reward,IsDone,NextState] = myStepFunction(Action,State)
    path = [2.00    1.00;
        3.00    3.00;
        4.00    4.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        10.00   10.00;
        11.75   10.75;
        12.00   10.00];

    robotGoal = path(end,:);

    x=State(1);
    y=State(2);
    ori=State(4);

    robot_position=[x, y];
    robotCurrentPose = [robot_position ori]';

    robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 0.6;
    controller.MaxAngularVelocity = 2;
    controller.LookaheadDistance = Action;

    goalRadius = 0.1;
    sampleTime = 0.1;
    [v, omega] = controller(robotCurrentPose);
    vel = derivative(robot, robotCurrentPose, [v omega]);
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
    [e_err,~]=calculate_e(path, robotCurrentPose(1:2));
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    cur_x=robotCurrentPose(1);
    cur_y=robotCurrentPose(2);

    NextState = [cur_x; cur_y; e_err; robotCurrentPose(3)];
    NextObs = NextState;

    PenaltyForFalling=-10;

    if distanceToGoal > goalRadius
        IsDone=e_err>3;
        if ~IsDone
            Reward = exp(-1*(e_err));
        else
            Reward = PenaltyForFalling;
        end
    else
        Reward=3;
    end
end

function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = norm(path(i,:) - current_position);
    end
    [e_err,index] = min(distances);
end