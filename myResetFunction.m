function [InitialObservation, InitialState]=myResetFunction()
    path = readmatrix('path_in.csv');   % 패스 로드

    % 위치 및 방향 초기화
    robotInitialLocation=path(1,:);
    x=robotInitialLocation(1);
    y=robotInitialLocation(2);
    ori = 1.732;

    % 횡방향 오차 계산
    [e_err,~]=calculate_e(path, robotInitialLocation);
    
    % State 반환
    InitialState=[x;y;e_err;ori];
    InitialObservation=InitialState;
end

% 모바일 로봇의 현재 위치와 패스간의 횡방향 오차 및 패스의 포인트 인덱스 반환
function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = sqrt((path(i,1)-current_position(1))^2+(path(i,2)-current_position(2))^2);
    end
    [e_err,index] = min(distances);
end