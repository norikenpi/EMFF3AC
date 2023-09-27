function [u, nearest_satellite_idx, histories, satellites, param] = controlAlgorithmDanilAC(histories, i, satellites, param, time)

    %startTime = datetime;
    [nearest_satellite_idx, histories, safety_distance, satellites] = findNearestSatellite(satellites, i, i, param, histories, time);
    
    
    
    
    
    
    %endTime = datetime;
    %executionTime = endTime - startTime;
    %timeString = [num2str(milliseconds(executionTime), '%.2f') ' ms'];
    %disp(['findNearestSatellite処理時間: ' timeString]);

    histories.pair_idx(int32(time/param.dt)+1, i) = nearest_satellite_idx;
    histories.pair_nearest_distance(int32(time/param.dt)+1, i) = norm(satellites{i}.position - satellites{nearest_satellite_idx}.position);


    % 加速度から必要な磁気モーメントを計算（相手の磁気モーメントが0だったら適宜設定）
    %ペアリングする衛星が存在＆＆衝突する可能性がある距離に存在しない＆＆最大距離より小さい
    %if converge == 0 && norm(relative_position) > param.safety_distance && norm(relative_position) < param.max_distance
    %if converge == 0 && norm(relative_position) < param.max_distance
        % C1を低減する必要がある場合、必要な加速度を計算
    
    %parameter = abs(satellites{i}.C1);

    parameter = 0;

    %relative_position = satellites{nearest_satellite_idx}.position - satellites{i}.position;
    %relative_velocity = satellites{nearest_satellite_idx}.velocity - satellites{i}.velocity;
    %relative_position_d = satellites{nearest_satellite_idx}.position_d - satellites{i}.position_d;
    %error = relative_position - relative_position_d;
    %histories.relative_position(int32(time/param.dt)+1, :, i) = error;
    %histories.relative_distance(int32(time/param.dt)+1, i) = norm(relative_position) - norm(relative_position_d);
    %衝突防止制御をするかどうか
    if safety_distance > 0
            %衛星間距離が小さすぎる場合
            u = calculateAvoidAcceleration(param, i, nearest_satellite_idx, satellites);
            
            %u = relativeFeedback(i, nearest_satellite_idx, satellites, param);
            histories.control_type(int32(time/param.dt)+1, i) = 99;
    elseif safety_distance == 0
        %param.C1_borderをめちゃくちゃ大きくすることで、常に目標相対位置誤差に基づく制御が可能になる。
        if parameter > param.control_border
            %C1を制御
            [u, satellites] = calculateRequiredAcceleration(param, satellites{i}.C1, i, nearest_satellite_idx, satellites);
            histories.control_type(int32(time/param.dt)+1, i) = 1;
            %param.pair_type = 'C1';

            %目標相対位置誤差を制御
            
            %u = relativeFeedback(i, nearest_satellite_idx, satellites, param);
            %histories.control_type(int32(time/param.dt)+1, i) = 2;
            %param.pair_type = 'velocity';

   
        elseif parameter <= param.control_border
            %目標相対位置誤差を制御
            
            [u, satellites, histories] = relativeFeedback(i, nearest_satellite_idx, satellites, param, histories);
            histories.control_type(int32(time/param.dt)+1, i) = 2;
            %param.pair_type = 'velocity';

        end
    end

    %{
    elseif norm(relative_position) <= param.safety_distance
        %一番近いやつとペア組めてないじゃん。
        disp("回避制御")
        disp(relative_position)
        u = calculateAvoidAcceleration(relative_position);
        disp(u)
    else
        u = [0;0;0];
    end
    %}
end
