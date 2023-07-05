function [position, velocity] = getSatellitePositionFusion(i, param)
    
    if i == 1 || i == 2 || i == 3 || i == 4
        mat = [[1,3];
                [2,4]];
    

        [ix, iy] = findValue(mat, i);
        rng(i)
        x = param.satellite_initial_distance * (ix) + rand() * param.initial_error; % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_initial_distance * (iy) + rand() * param.initial_error;
        z = 0 + rand() * param.initial_error;
        vx = 0;
        vy = 0;
        vz = 0;
    elseif i == 5 || i == 6 || i == 7 || i == 8
        i = i - 4;
        [ix, iy, iz] = ind2sub(param.start_state, i);
        rng(i)
        x = param.satellite_initial_distance * (ix) + rand() * param.initial_error - 0.3; % -0.15から0.15の範囲で衛星を配置
        y = param.satellite_initial_distance * 1.5 + rand() * param.initial_error;
        z = param.satellite_initial_distance * (iz) + rand() * param.initial_error;
        vx = 0.001;
        vy = 0;
        vz = 0;
    end
    position = [x; y; z];
    velocity = [vx; vy; vz];
end
