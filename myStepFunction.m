function [NextObs,Reward,IsDone,NextState] = myStepFunction(Action,State)
    path = readmatrix('path_in.csv');   % 패스 로드

    % State로부터 모바일 로봇의 위치와 방향 지정
    x=State(1);
    y=State(2);
    ori=State(4);
    robot_position=[x, y];
    robotCurrentPose = [robot_position ori]';

    % 로봇 모델 지정 및 주행 파라미터 설정
    robot = bicycleKinematics(WheelBase=1.212,MaxSteeringAngle=0.349,...
        VehicleInputs="VehicleSpeedSteeringAngle",VehicleSpeedRange=[0, 11.11]);
    controller = controllerPurePursuit;
    controller.Waypoints = path;
    controller.DesiredLinearVelocity =10.0;
    controller.MaxAngularVelocity = 2.82;
    controller.LookaheadDistance = Action;  % 모델의 예측값을 LAD값으로 지정

    % 도착 판단 거리와 샘플 타임 지정
    goalRadius = 1.0;
    sampleTime = 0.1;

    % 모바일 로봇의 다음 위치 및 방향 계산
    [v, omega] = controller(robotCurrentPose);
    vel = derivative(robot, robotCurrentPose, [v omega]);
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
    % 횡방향 오차 계산
    [e_err,~]=calculate_e(path, robotCurrentPose(1:2));

    % 도착지점과의 거리 측정
    robotGoal = path(end,:); 
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % 다음 상태값 지정
    cur_x=robotCurrentPose(1);
    cur_y=robotCurrentPose(2);
    NextState = [cur_x; cur_y; e_err; robotCurrentPose(3)];
    NextObs = NextState;


% 사용하지 않을 보상함수에 주석처리

% V1보상함수
    
    PenaltyForFalling=-1000000; % 경로이탈 패널티
    if distanceToGoal > goalRadius  % 에피소드 도착 종료조건
        % 경로이탈 여부 확인
        IsDone=e_err>2.0;
        if ~IsDone
            Reward = 50*(-exp(1.5*e_err)+1);    % 스탭 보상함수
        else
            Reward = PenaltyForFalling;
        end
    else
        Reward=10000;
        IsDone = true;
    end
end

% V2보상함수
    
%     PenaltyForFalling=-100000;  % 경로이탈 패널티
%     if distanceToGoal > goalRadius  % 에피소드 도착 종료조건
%         % 경로이탈 여부 확인
%         IsDone=e_err>2.0;   
%         if ~IsDone
%             Reward = -1000*e_err;               % 스탭 보상함수
%         else
%             Reward = PenaltyForFalling;
%         end
%     else
%         Reward=10000;
%         IsDone = true;
%     end
% end




% 모바일 로봇의 현재 위치와 패스간의 횡방향 오차 및 패스의 포인트 인덱스 반환
function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = sqrt((path(i,1)-current_position(1))^2+(path(i,2)-current_position(2))^2);
    end
    [e_err,index] = min(distances);
end