% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatelliteDesiredPosition(i, param)
    mat = [[1,9,13,5];
           [2,10,14,6];
           [3,11,15,7];
           [4,12,16,8]];

    [ix, iy] = findValue(mat, i);
    rng(i)
    x = 0.15 * (ix); % -0.15から0.15の範囲で衛星を配置
    y = 0.15 * (iy);
    z = 0;
    %x = 0.15 * (ix - (m+1)/2);
    %y = 0.15 * (iy - (n+1)/2);
    %z = 0.15 * (iz - (p+1)/2);
    % 3次元ベクトルに変換して返す
    pos = [x; y; z];
    vel = [0; 0; 0];
end