% シミュレーションの各タイムステップで衛星の状態を更新する関数
function [satellites, histories] = simulateTimeStep(satellites, histories, param)
    % 以下の手順を実行する:
    % 1. 最も近いドリフト衛星のインデックスを取得
    % 2. C1を低減するような必要な加速度を計算
    % 3. 加速度から必要な磁気モーメントを計算
    % 4. 各衛星に働く磁気トルクと磁気力を計算
    % 5. 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算
    % 6. 位置と力の履歴を更新

    %各衛星に関して制御アルゴリズムを実行して、各衛星の磁気モーメントを計算。
    for i = 1:param.N
        [u, nearest_drift_satellite_idx] = controlAlgorithm(histories, i, satellites, param);

        %uに基づいて衛星の磁気モーメントを計算
        satellites{i}.magnetic_moment = setSatelliteDipole(satellites, u, i, nearest_drift_satellite_idx);
        
        %計算された磁気モーメントに基づいて、磁力を計算しなおす（磁気モーメントには上限があるから）
        u_real = magneticForceSatellite(i, nearest_drift_satellite_idx, satellites)/satellites{i}.mass;

        histories.u_histories{i} = [histories.u_histories{i}, u];
        histories.u_real_histories{i} = [histories.u_real_histories{i}, u_real];
        histories.pair_idx{i} = [histories.pair_idx{i}, nearest_drift_satellite_idx];
    end
    
     % 各衛星に発生した磁気トルクと磁気力を計算
    [magnetic_forces, magnetic_torques] = calculateF_total(satellites, param);

    % 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算。
    satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces);
    %force_sum = zeros(3,1);



    % 各衛星に関して位置と力の履歴を更新
    for i = 1:param.N
        histories.position_histories{i} = [histories.position_histories{i}, satellites{i}.position];
        histories.force_histories{i} = [histories.force_histories{i}, magnetic_forces{i}];
        histories.magnetic_moment_histories{i} = [histories.magnetic_moment_histories{i}, satellites{i}.magnetic_moment];
        histories.magnetic_forces_histories{i} = [histories.magnetic_forces_histories{i}, magnetic_forces{i}];
     
        %force_sum = force_sum + magnetic_forces{i};
    end
end

