%衛星間の距離を10cm以上離した衛星配置を行う
%C1~C6 rand([-0.1; 0.1])
function [pos, vel] = getSatellitePositionRandom3(i, satellites, param)
    seedj = param.j;
   

    if i == 1
        relative_position = zeros(3,1);
        %relative_velocity = zeros(3,1);
    else
        relative_position = satellites{i-1}.position;
        %relative_velocity = satellites{i-1}.velocity;
    end

    if i > 2
        relative_distance_min = 0;
        %衛星間の最小距離が規定の長さよりも短くなるように設定する。
        while relative_distance_min < (param.radius * 3)
            seedj = seedj + 1;
            rng(seedj+i);
            vector = rand(3, 1) - 0.5;
            distance = param.radius* 3 * vector/norm(vector);
            pos = relative_position + distance;
            
            %最も近い距離にある衛星を探す。
            relative_distance_min = 1000;
            for j = 1:(i-2)
                relative_distance = norm(pos - satellites{j}.position);
                if relative_distance < relative_distance_min
                    relative_distance_min = relative_distance;
                end
            end
        end
    elseif i <= 2
        relative_distance_min = 100;
        rng(seedj+i);
        vector = rand(3, 1) - 0.5;
        distance = param.radius* 3 * vector/norm(vector);
        pos = relative_position + distance;
    end


    seedj = param.j;
    rng(i+seedj);
    mean_vel = param.mean_vel; 
    std_vel = param.std_vel;

    vel = normrnd(mean_vel, sqrt(std_vel), 1, 3).';

    %{
    for i = 1:param.N
        % 1メートル以上離れた位置にランダムな点を生成する
        while true
            point = rand(1, 3) * 20 - 10; % [-10, 10]の範囲でランダムな座標を生成する

            % 他の点との距離を計算する
            distances = sqrt(sum((points(1:i-1, :) - point).^2, 2));
            if all(distances >= 1)
                break;
            end
        end
        
        points(i, :) = point;
    end
    %}
end
