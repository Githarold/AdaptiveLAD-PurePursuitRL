function [NextObs,Reward,IsDone,NextState] = myStepFunction(Action,State)
path = [-20.00   -20.00;        -19.50   -19.01;        -19.00   -18.05;        -18.50   -17.11;        -18.00   -16.20;        -17.50   -15.31;        -17.00   -14.45;        -16.50   -13.61;        -16.00   -12.80;        -15.50   -12.01;        -15.00   -11.25;        -14.50   -10.51;        -14.00    -9.80;        -13.50    -9.11;        -13.00    -8.45;        -12.50    -7.81;        -12.00    -7.20;        -11.50    -6.61;        -11.00    -6.05;        -10.50    -5.51;        -10.00    -5.00;        -9.50     -4.51;        -9.00     -4.05;        -8.50     -3.61;        -8.00     -3.20;        -7.50     -2.81;        -7.00     -2.45;        -6.50     -2.11;        -6.00     -1.80;        -5.50     -1.51;        -5.00     -1.25;        -4.50     -1.01;        -4.00     -0.80;        -3.50     -0.61;        -3.00     -0.45;        -2.50     -0.31;        -2.00     -0.20;        -1.50     -0.11;        -1.00     -0.05;        -0.50     -0.01;         0.00      0.00;         0.50     -0.01;         1.00     -0.05;         1.50     -0.11;         2.00     -0.20;         2.50     -0.31;         3.00     -0.45;         3.50     -0.61;         4.00     -0.80;         4.50     -1.01;         5.00     -1.25;         5.50     -1.51;         6.00     -1.80;         6.50     -2.11;         7.00     -2.45;         7.50     -2.81;         8.00     -3.20;         8.50     -3.61;         9.00     -4.05;         9.50     -4.51;         10.00    -5.00;         10.50    -5.51;         11.00    -6.05;         11.50    -6.61;         12.00    -7.20;         12.50    -7.81;         13.00    -8.45;         13.50    -9.11;         14.00    -9.80;         14.50   -10.51;         15.00   -11.25;         15.50   -12.01;         16.00   -12.80;         16.50   -13.61;         17.00   -14.45;         17.50   -15.31;         18.00   -16.20;         18.50   -17.11;         19.00   -18.05;         19.50   -19.01;         20.00   -20.00];


    robotGoal = path(end,:);

    x=State(1);
    y=State(2);
    ori=State(4);

    robot_position=[x, y];
    robotCurrentPose = [robot_position ori]';

    robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity = 3;
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
        IsDone = true;
    end
end

function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = sqrt((path(i,1)-current_position(1))^2+(path(i,2)-current_position(2))^2);
    end
    [e_err,index] = min(distances);
end