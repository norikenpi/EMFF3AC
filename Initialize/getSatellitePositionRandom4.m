%既定の範囲内で衛星の初期位置を設定。Cベース
%C1~C6 rand([-0.1; 0.1])
function [pos, vel] = getSatellitePositionRandom4(i, satellites, param)
    seedj = param.j;
    rng(i+seedj);
    mean_pos = param.mean_pos;
    std_pos = param.std_pos;
    mean_vel = param.mean_vel; 
    std_vel = param.std_vel;
    pos = normrnd(mean_pos, sqrt(std_pos), 1, 3).';
    % 3つの衛星の位置を生成
    if i > 1
        relative_distance_min = 0;
        %衛星間の最小距離が規定の長さよりも短くなるように設定する。
        while relative_distance_min < (param.radius * 3)
            seedj = seedj + 1;
            rng(seedj+i);
            pos = normrnd(mean_pos, sqrt(std_pos), 1, 3).';
            
            %最も近い距離にある衛星を探す。
            relative_distance_min = 1000;
            for j = 1:(i-1)
                relative_distance = norm(pos - satellites{j}.position);
                if relative_distance < relative_distance_min
                    relative_distance_min = relative_distance;
                end
            end
        end
    end

    



    % 3つの衛星の速度を生成
    vel = normrnd(mean_vel, sqrt(std_vel), 1, 3).';

end
