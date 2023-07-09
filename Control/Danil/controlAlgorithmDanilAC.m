function [u, nearest_satellite_idx, histories] = controlAlgorithmDanilAC(histories, i, satellites, param, time)
    [nearest_satellite_idx, C1] = findNearestSatellite(satellites, i, i, param);
    histories.C1_histories(int32(time/param.dt)+1, i) = C1;
    %histories.pair_idx{i} = [histories.pair_idx{i}, nearest_drift_satellite_idx];

    % 加速度から必要な磁気モーメントを計算（相手の磁気モーメントが0だったら適宜設定）
    %ペアリングする衛星が存在＆＆衝突する可能性がある距離に存在しない＆＆最大距離より小さい
    %if converge == 0 && norm(relative_position) > param.safety_distance && norm(relative_position) < param.max_distance
    %if converge == 0 && norm(relative_position) < param.max_distance
        % C1を低減する必要がある場合、必要な加速度を計算

    
    %param.C1_borderをめちゃくちゃ大きくすることで、常に目標相対位置誤差に基づく制御が可能になる。
    if norm(C1) > param.C1_border
        %C1を制御
        u = calculateRequiredAcceleration(param, C1, i, nearest_satellite_idx, satellites);
        histories.control_type(int32(time/param.dt)+1, i) = 1;
    elseif norm(C1) <= param.C1_border
        %目標相対位置誤差を制御
        u = relativeFeedback(i, nearest_satellite_idx, satellites, param);
        histories.control_type(int32(time/param.dt)+1, i) = 2;
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
