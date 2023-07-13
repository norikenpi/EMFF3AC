% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatelliteDesiredPosition(i, param)

    mat = [[1,9,13,5];
           [2,10,14,6];
           [3,11,15,7];
           [4,12,16,8]];

    
    mat = [[1,3];
           [2,4]];
    %{
    mat = [[1,5];
           [2,6];
           [3,7];
           [4,8]];
    %}
    
    

    [ix, iy] = findValue(mat, i);
    rng(i)
    x = param.satellite_desired_distance * (ix); % -0.15から0.15の範囲で衛星を配置
    y = param.satellite_desired_distance * (iy);
    z = 0;
    if i == 5
        x = param.satellite_desired_distance * (1); % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_desired_distance * (0);
        z = 0;
    end
    if i == 6
        x = param.satellite_desired_distance * (3); % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_desired_distance * (1);
        z = 0;
    end
    if i == 7
        x = param.satellite_desired_distance * (0); % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_desired_distance * (2);
        z = 0;
    end
    if i == 8
        x = param.satellite_desired_distance * (2); % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_desired_distance * (3);
        z = 0;
    end
    %pos = [x + rand() * param.initial_error ; y + rand() * param.initial_error; z + rand() * param.initial_error];
    pos = [x ; y ; z];
    vel = [0; 0; 0];
end