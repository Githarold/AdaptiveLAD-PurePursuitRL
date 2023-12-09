% 차량의 상태 데이터를 불러옴
a=experience.Observation.CarState.Data(:,:,1);
iter=size(experience.Observation.CarState.Time);
iter=iter(1);

% 경로 데이터를 CSV 파일에서 읽어옴
path = readmatrix('path_in.csv');

% 경로의 최소 및 최대 x, y 좌표를 계산
x_min =min(path(:,1));
y_min =min(path(:,2));
x_max= max(path(:,1));
y_max= max(path(:,2));

% 로봇의 초기 위치 설정
robotInitialLocation=[a(1),a(2)];

% 로봇의 목표 위치 설정
robotGoal = path(end,:);

% 로봇의 초기 방향 설정
initialOrientation = 1.732;

% 로봇의 현재 위치 및 방향
robotCurrentPose = [robotInitialLocation initialOrientation]';

% 자전거 운동학을 사용한 로봇 모델 생성
robot = bicycleKinematics(WheelBase=1.212,MaxSteeringAngle=0.349,VehicleInputs="VehicleSpeedSteeringAngle",VehicleSpeedRange=[0, 11.11]);

% 첫 번째 그래프 (차량의 이동경로)
subplot(6, 1, [1 4]); % 3개의 행과 1개의 열을 가진 subplot의 첫 번째 위치 설정
light;
plot(path(:,1), path(:,2), '-');
xlim([x_min-5 x_max+5]);
ylim([y_min-5 y_max+5]);
title('차량 이동경로');

% 두 번째 그래프 (조향 각도)
subplot(6, 1, 5); % 같은 subplot의 두 번째 위치 설정
yy = experience.Action.CarAction.Data(:);
plot(1:iter-1, yy);
title('LAD');
xlabel('step');
ylabel('(m)');

% 세 번째 그래프 (횡방향 오차)
subplot(6, 1, 6); % 같은 subplot의 세 번째 위치 설정
yy = experience.Observation.CarState.Data(3,:,:);
plot(1:iter, yy(:,:));
hold on;
err_mean = mean(yy(:,:));
err_mean_plot = ones(1, iter) * err_mean;
plot(1:iter, err_mean_plot, '--');
hold off;
title('횡방향오차');
xlabel('step');
ylabel('(m)');
legend('횡방향오차', '평균 횡방향오차', 'Location', 'southwest');

% 순수추격 제어기 생성 및 설정
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 10.0;
controller.MaxAngularVelocity = 2.82;

% 목표 반경 설정
goalRadius = 0.1;
% 목표까지의 거리 계산
distanceToGoal = norm(robotInitialLocation - robotGoal);
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% 시뮬레이션 루프 시작
% figure
reward=0;
frameSize = 3;
i=1;
iter=size(experience.Observation.CarState.Time);
iter=iter(1);
robotTrajectory = zeros(2, iter);

% 입력대기
str = input('시뮬레이션을 실행하려면 아무키나 입력하세요.', 's');

while i<iter
    % 조향 각도 설정
    b=experience.Action.CarAction.Data(i);
    controller.LookaheadDistance = b;
    [v, omega] = controller(robotCurrentPose);

    % 로봇의 속도 업데이트
    vel = derivative(robot, robotCurrentPose, [v omega]);
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 

    % 목표까지의 거리 업데이트
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % 현재 위치 저장
    robotTrajectory(:, i) = robotCurrentPose(1:2);

    % 모바일 로봇 Plot
    subplot(6, 1, [1 4]);
    cla; 
    plot(path(:,1), path(:,2), "-");
    hold on;
    plot(robotTrajectory(1, 1:i), robotTrajectory(2, 1:i), 'r--');
    legend("로봇이 추종해야 하는 경로", "로봇이 실제 움직인 경로", 'Location', 'southwest')
    hold all;
    [e_err,~]=calculate_e(path, robotCurrentPose(1:2));
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View", "2D", "FrameSize", frameSize);
    light;
    xlim([x_min-5 x_max+5]);
    ylim([y_min-5 y_max+5]);

    % LAD Plot
    subplot(6, 1, 5);
    cla; 
    plot(1:iter-1, experience.Action.CarAction.Data(:));
    hold on;
    plot(i, experience.Action.CarAction.Data(i), 'r.', 'MarkerSize', 15);
    title('LAD');
    xlabel('step');
    ylabel('(m)');

    % 횡방향오차 Plot
    subplot(6, 1, 6);
    cla;
    plot(1:iter, yy(:,:));
    hold on; 
    plot(1:iter, err_mean_plot, '--');
    plot(i, yy(:,i), 'r.', 'MarkerSize', 15);
    hold off;
    title('횡방향오차');
    xlabel('step');
    ylabel('(m)');
    legend('횡방향오차', '평균 횡방향오차', 'Location', 'southwest');

    % 보상 업데이트
    reward=reward+exp(-1*(e_err));
    i=i+1;
    waitfor(vizRate);
end

% 경로와 현재 위치에 따른 횡방향 오차를 계산하는 함수
function [e_err, index,distances]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);
    for i = 1:size(path, 1)
        distances(i) = sqrt((path(i,1)-current_position(1))^2+(path(i,2)-current_position(2))^2);
    end
    [e_err,index] = min(distances);
end
