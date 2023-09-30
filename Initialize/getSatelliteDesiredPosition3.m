% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatelliteDesiredPosition3(i, param)
    rng(i)

    if i == 1
        x = param.satellite_desired_distance/2; % -0.15から0.15の範囲で衛星を配置
        y = 0;
        z = 0;
    elseif i == 2
        x = -param.satellite_desired_distance/2; % -0.15から0.15の範囲で衛星を配置
        y = 0;
        z = 0;
    elseif i == 3
        x = 0; % -0.15から0.15の範囲で衛星を配置
        y = -param.satellite_desired_distance*sqrt(3)/2;
        z = 0;
    end

    vx = 0;
    vy = 0;
    vz = 0;

    pos = [x; y; z];
    vel = [vx; vy; vz];
end