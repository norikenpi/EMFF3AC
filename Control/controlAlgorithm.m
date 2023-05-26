function [u, nearest_drift_satellite_idx] = controlAlgorithm(histories, i, satellites, param)
    [nearest_drift_satellite_idx, C1, relative_position, converge, relative_position_min] = findNearestDriftSatellite(satellites, param, i);
    histories.C1_histories{i} = [histories.C1_histories{i}, C1];
    histories.pair_idx{i} = [histories.pair_idx{i}, nearest_drift_satellite_idx];

    % 加速度から必要な磁気モーメントを計算（相手の磁気モーメントが0だったら適宜設定）
    %ペアリングする衛星が存在＆＆衝突する可能性がある距離に存在しない＆＆最大距離より小さい
    %if converge == 0 && norm(relative_position) > param.safety_distance && norm(relative_position) < param.max_distance
    if converge == 0 && norm(relative_position) < param.max_distance
        % C1を低減する必要がある場合、必要な加速度を計算
        
        if norm(relative_position_min) == norm(relative_position)
            u = calculateRequiredAcceleration(param, C1);
        
        else
            
            if norm(relative_position_min) > param.min_distance_nopair
                u = calculateRequiredAcceleration(param, C1);
            else 
                u = [0;0;0];
            end
            
        end
        
    
    elseif norm(relative_position) <= param.safety_distance
        %一番近いやつとペア組めてないじゃん。
        disp("回避制御")
        disp(relative_position)
        u = calculateAvoidAcceleration(relative_position);
        disp(u)
    else
        u = [0;0;0];
    end
end
