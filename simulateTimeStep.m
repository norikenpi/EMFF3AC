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
        %[u, pair_satellite_idx] = controlAlgorithmDanil(histories, i, satellites, param);
        
        
        [u, pair_satellite_idx] = controlAlgorithmStepbystep(histories, i, satellites, param, time);
        
        %シミュレーションタイムステップの間は変えない。
        %uに基づいて衛星の磁気モーメントを計算
        if mod(time, param.time_step) == 0
            satellites{i}.magnetic_moment = setSatelliteDipole(satellites, u, i, pair_satellite_idx);
        end
        
        %計算された磁気モーメントに基づいて、磁力を計算しなおす（磁気モーメントには上限があるから）

        u_real = magneticForceSatellite(i, pair_satellite_idx, satellites, param)/satellites{i}.mass;

        histories.u_histories(int32(time/param.dt)+1, :, i) = u;
        histories.u_real_histories(int32(time/param.dt)+1, :, i) = u_real;
        histories.pair_idx(int32(time/param.dt)+1, i) = pair_satellite_idx;
        %fprintf('final pair %d %d\n', i, pair_satellite_idx)
    end
    
    
     % 各衛星に発生した磁気トルクと磁気力を計算
    [magnetic_forces, magnetic_torques] = calculateF_total(satellites, param);

    % 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算。
    satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces);

    %紐がつながっているときの制御
    %各衛星の相対距離を計算して、目標相対距離以上になった場合、相対位置ベクトル方向の速度ベクトルのうち、相対位置ベクトルの逆向きにする。
    %satellites = updateSatelliteStatesString(satellites, param);



    % 各衛星に関して位置と力の履歴を更新
    for i = 1:param.N
        histories.position_histories(int32(time/param.dt)+1, :, i) = satellites{i}.position;
        histories.force_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i};
        histories.magnetic_moment_histories(int32(time/param.dt)+1, :, i) = satellites{i}.magnetic_moment;
        histories.magnetic_forces_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i};
     
        %force_sum = force_sum + magnetic_forces{i};
    end
end

