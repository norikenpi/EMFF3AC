function [pos, vel] = getSatelliteDesiredPosition1000(i, param)
    
    %16衛星でテストするための初期条件
    [m, n, p] = size(reshape(1:param.N, param.N/4, 2, 2));

    %4衛星でテストするための初期条件
    %[m, n, p] = size(reshape(1:param.N, 2, 2, 1));

    m = 18;
    n = 12;
    p = 1;

    % インデックスiに対応するx, y, z座標を計算
    [ix, iy, iz] = ind2sub([m, n, p], i);
    rng(i)
    x = param.satellite_initial_distance * (ix) + rand() * param.initial_error; % -0.15から0.15の範囲で衛星を配置
    y = param.satellite_initial_distance * (iy) + rand() * param.initial_error;
    z = param.satellite_initial_distance * (iz) + rand() * param.initial_error;

    pos = [x; y; z];
    vel = [0; 0; 0];
end