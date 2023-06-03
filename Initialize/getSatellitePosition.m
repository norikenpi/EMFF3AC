% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatellitePosition(i, param)
    
    [m, n, p] = size(reshape(1:param.N, param.N/4, 2, 2));
    % インデックスiに対応するx, y, z座標を計算
    [ix, iy, iz] = ind2sub([m, n, p], i);
    rng(i)
    x = 0.15 * (ix) + rand() * param.initial_error - 0.02; % -0.15から0.15の範囲で衛星を配置
    y = 0.15 * (iy) + rand() * param.initial_error - 0.02;
    z = 0.15 * (iz) + rand() * param.initial_error - 0.02;
    %x = 0.15 * (ix - (m+1)/2);
    %y = 0.15 * (iy - (n+1)/2);
    %z = 0.15 * (iz - (p+1)/2);
    % 3次元ベクトルに変換して返す
    pos = [x; y; z];
    vel = [0; 0; 0];
end
