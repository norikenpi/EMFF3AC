% 衛星を(param.N/4)×2×2の配列に配置する
function [pos, vel] = getSatelliteDesiredPosition(i, param)
    mat = [[1,9,13,5];
           [2,10,14,6];
           [3,11,15,7];
           [4,12,16,8]];
    %{
    mat = [[1,3];
           [2,4]];
    %}

    [ix, iy] = findValue(mat, i);
    rng(i)
    x = param.satellite_desired_distance * (ix); % -0.15から0.15の範囲で衛星を配置
    y = param.satellite_desired_distance * (iy);
    z = 0;
    pos = [x; y; z];
    vel = [0; 0; 0];
end