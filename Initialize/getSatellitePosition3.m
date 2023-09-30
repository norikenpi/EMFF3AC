% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatellitePosition3(i, param)
    rng(i*3)

    if i == 1
        x = param.satellite_desired_distance/2 + rand() * param.initial_position_error; % -0.15から0.15の範囲で衛星を配置
        y = rand() * param.initial_position_error;
        z = 0;
    elseif i == 2
        x = -param.satellite_desired_distance/2 + rand() * param.initial_position_error; % -0.15から0.15の範囲で衛星を配置
        y = rand() * param.initial_position_error;
        z = 0;
    elseif i == 3
        x = rand() * param.initial_position_error; % -0.15から0.15の範囲で衛星を配置
        y = -param.satellite_desired_distance*sqrt(3)/2 + rand() * param.initial_position_error;
        z = 0;
    end

    vx = rand() * param.initial_velocity_error;
    vy = rand() * param.initial_velocity_error;
    vz = rand() * param.initial_velocity_error;
    vx = 0;
    vy = 0;
    vz = 0;

    pos = [x; y; z];
    vel = [vx; vy; vz];
end