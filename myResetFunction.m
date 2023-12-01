function [InitialObservation, InitialState]=myResetFunction()
    path = [2.00    1.00;
        3.00    3.00;
        4.00    4.00;
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        10.00   10.00;
        11.75   10.75;
        12.00   10.00];
    robotInitialLocation=path(1,:)+[rand()*0.5-0.5 rand()*0.5-0.5];
    ori = 0;
    [e_err,~]=calculate_e(path, robotInitialLocation);
    x=robotInitialLocation(1);
    y=robotInitialLocation(2);
    InitialState=[x;y;e_err;ori];
    InitialObservation=InitialState;
end

function [e_err, index]=calculate_e(path, current_position)
    distances = zeros(size(path, 1), 1);

    % 각 path 포인트에 대해 거리 계산
    for i = 1:size(path, 1)
        distances(i) = norm(path(i,:) - current_position);
    end
    [e_err,index] = min(distances);
end