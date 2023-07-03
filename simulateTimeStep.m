% シミュレーションの各タイムステップで衛星の状態を更新する関数
function [satellites, histories] = simulateTimeStep(satellites, histories, param, time)
    % 以下の手順を実行する:
    % 1. 最も近いドリフト衛星のインデックスを取得
    % 2. C1を低減するような必要な加速度を計算
    % 3. 加速度から必要な磁気モーメントを計算
    % 4. 各衛星に働く磁気トルクと磁気力を計算
    % 5. 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算
    % 6. 位置と力の履歴を更新

    %各衛星に関して制御アルゴリズムを実行して、各衛星の磁気モーメントを計算。
    for i = 1:param.N

        %Danil式かStepbystepのどちらかを選択。
        %[u, pair_satellite_idx] = controlAlgorithmDanil(histories, i, satellites, param, time);
        [u, pair_satellite_idx] = controlAlgorithmDanilnotC1(histories, i, satellites, param, time);

        %[u, pair_satellite_idx] = controlAlgorithmDanilAC(histories, i, satellites, param, time);

        
        % StepbyStep方式
        %交流の場合、uは行列形式
        %[u, pair_satellite_idx] = controlAlgorithmStepbystep(histories, i, satellites, param, time);

        
        % シミュレーションタイムステップの間は電流の大きさを変えない。
        % uに基づいて衛星の磁気モーメントを計算
        % 各衛星の正規化後の磁気モーメントと所望の磁気ダイポールを計算。
        if mod(time, param.time_step) == 0
        %制御周期おきに磁気ダイポール計算をする。setSaetelliteDipoleの中で交流と直流で処理を分けてる。
            [satellites{i}.magnetic_moment, magnetic_moment_req] = setSatelliteDipole(satellites, u, i, pair_satellite_idx, histories, time, param);
        else 
        %制御周期の間は所望磁気モーメントは変わらない。
            magnetic_moment_req = histories.magnetic_moment_req_histories(int32(time/param.dt), :, i);
        end 
        
        
        
        if param.current_type == "DC"
            %計算された磁気モーメントに基づいて、実際に発生する磁力を計算しなおす（磁気モーメントには上限があるから）
            %交流の場合、直流の場合とでこの関数の中で計算し分けられる。
            %u_real = magneticForceSatellite(i, pair_satellite_idx, satellites, param)/satellites{i}.mass;
            histories.magnetic_moment_req_histories(int32(time/param.dt)+1, :, i) = magnetic_moment_req;
            histories.u_histories(int32(time/param.dt)+1, :, i) = u;
            %histories.u_real_histories(int32(time/param.dt)+1, :, i) = u_real;
            disp("paire")
            disp(pair_satellite_idx)
            histories.pair_idx(int32(time/param.dt)+1, i) = pair_satellite_idx;
        end
    end
    
    
     % 各衛星に発生した磁気トルクと磁力を計算
    [magnetic_forces, magnetic_torques] = calculateF_total(satellites, param, time, histories);
    


    % 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算。
    satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces);

    %紐がつながっているときの制御
    %各衛星の相対距離を計算して、目標相対距離以上になった場合、相対位置ベクトル方向の速度ベクトルのうち、相対位置ベクトルの逆向きにする。
    %satellites = updateSatelliteStatesString(satellites, param);



    % 各衛星に関して位置と力の履歴を更新
    for i = 1:param.N
        histories.position_histories(int32(time/param.dt)+1, :, i) = satellites{i}.position;
        histories.velocity_histories(int32(time/param.dt)+1, :, i) = satellites{i}.velocity;
        histories.force_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i}; 
        histories.magnetic_forces_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i};
        if param.current_type == "DC"
             histories.magnetic_moment_histories(int32(time/param.dt)+1, :, i) = satellites{i}.magnetic_moment;
        end
    end
end

