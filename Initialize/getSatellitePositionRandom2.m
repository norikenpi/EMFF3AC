%既定の範囲内で衛星の初期位置を設定。Cベース
%C1~C6 rand([-0.1; 0.1])
function [pos, vel] = getSatellitePositionRandom2(i, satellites, param)
    j = param.j;
    rng(i+j);
    mean_pos = param.mean_pos;
    std_pos = param.std_pos;
    mean_vel = param.mean_vel; 
    std_vel = param.std_vel;

    % 3つの衛星の位置を生成
    pos = normrnd(mean_pos, sqrt(std_pos), 1, 3).';

    % 3つの衛星の速度を生成
    vel = normrnd(mean_vel, sqrt(std_vel), 1, 3).';

end
