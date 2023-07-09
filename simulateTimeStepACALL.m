% シミュレーションの各タイムステップで衛星の状態を更新する関数
function [satellites, histories] = simulateTimeStepACALL(satellites, histories, param, time)
    %param.setの数だけ、繰り返す。
    for idx = 1:param.N
        satellites{idx}.magnetic_moment = zeros(3, size(param.frequency, 1));
    end
    for idx = 1:param.N

        %Danil式かStepbystepのどちらかを選択。
        %[u, pair_satellite_idx] = controlAlgorithmDanil(histories, i, satellites, param);
        
        [u, nearest_satellite_idx, histories] = controlAlgorithmDanilAC(histories, idx, satellites, param, time);
        histories.pair_idx(int32(time/param.dt)+1, idx) = nearest_satellite_idx;
        % StepbyStep方式
        %iは衛星ペア
        %pair = param.set_AC(i, :);
        %u = controlAlgorithmAC(histories, pair, satellites, param, time);

        
        % シミュレーションタイムステップの間は電流の大きさを変えない。
        % uに基づいて衛星の磁気モーメントを計算
        % 各衛星の正規化後の磁気モーメントと所望の磁気ダイポールを計算。
        if mod(time, param.time_step) == 0
        %制御周期おきに磁気ダイポール計算をする。setSaetelliteDipoleの中で交流と直流で処理を分けてる。
            %satellites = setSatelliteDipoleAC(satellites, u, idx, histories, time, param);
            satellites = setSatelliteDipoleAC2(satellites, u, idx, nearest_satellite_idx, histories, time, param);
        else 
        %制御周期の間は所望磁気モーメントは変わらない。
            magnetic_moment_req = histories.magnetic_moment_req_histories(int32(time/param.dt), :, idx);
        end 
        
       
    end

    %各衛星の磁気ダイポールモーメントの上限に基づいて大きさを変更
    %satellites = adjustMagneticMoment(satellites, param); 
    %各衛星の磁気ダイポールモーメントの上限に基づいて、要求電流の割合に応じて電流を分配。
    satellites = adjustMagneticMomentAC(satellites, param); 
     % 各衛星に発生した磁気トルクと磁力を計算
    [magnetic_forces, magnetic_torques, histories] = calculateF_totalAC(satellites, param, time, histories);

    % 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算。
    satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces);

    % 各衛星に関して位置と力の履歴を更新
    for idx = 1:param.N
        histories.position_histories(int32(time/param.dt)+1, :, idx) = satellites{idx}.position;
        histories.force_histories(int32(time/param.dt)+1, :, idx) = magnetic_forces{idx};
        
        histories.magnetic_forces_histories(int32(time/param.dt)+1, :, idx) = magnetic_forces{idx};
        magnetic_moment_sum = 0;
        for freq_i = 1:size(param.frequency, 1)
            magnetic_moment_sum = magnetic_moment_sum + norm(satellites{idx}.magnetic_moment(:,freq_i));
        end
        histories.current_histories(int32(time/param.dt)+1, idx) = magnetic_moment_sum/(pi * param.radius^2 * param.coilN);
        
    end
end