% シミュレーションの各タイムステップで衛星の状態を更新する関数
function [satellites, histories] = simulateTimeStepAC(satellites, histories, param, time)
    %param.setの数だけ、繰り返す。
    for i = 1:size(param.set,1)

        %Danil式かStepbystepのどちらかを選択。
        %[u, pair_satellite_idx] = controlAlgorithmDanil(histories, i, satellites, param);
        
        % StepbyStep方式
        %iは衛星ペア
        pair = param.set_AC(i, :);
        
        
        u = controlAlgorithmAC(histories, pair, satellites, param, time);

        
        % シミュレーションタイムステップの間は電流の大きさを変えない。
        % uに基づいて衛星の磁気モーメントを計算
        % 各衛星の正規化後の磁気モーメントと所望の磁気ダイポールを計算。
        if mod(time, param.time_step) == 0
        %制御周期おきに磁気ダイポール計算をする。setSaetelliteDipoleの中で交流と直流で処理を分けてる。
            
            satellites = setSatelliteDipoleAC(satellites, u, i, histories, time, param);
        else 
        %制御周期の間は所望磁気モーメントは変わらない。
            magnetic_moment_req = histories.magnetic_moment_req_histories(int32(time/param.dt), :, i);
        end 
        
       
    end

    %各衛星の磁気ダイポールモーメントの上限に基づいて大きさを変更
    satellites = adjustMagneticMoment(satellites, param);    
     % 各衛星に発生した磁気トルクと磁力を計算
    [magnetic_forces, magnetic_torques] = calculateF_totalAC(satellites, param, time);
    
    %disp("position force")
    %disp(magnetic_forces{1})
    %disp(magnetic_forces{2})
    %disp(magnetic_forces{3})
    %disp(magnetic_forces{4})

    % 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を計算。
    satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces);
    %disp("position update")
    %disp(satellites{1}.position)
    %disp(satellites{2}.position)
    %disp(satellites{3}.position)
    %disp(satellites{4}.position)



    % 各衛星に関して位置と力の履歴を更新
    for i = 1:param.N
        histories.position_histories(int32(time/param.dt)+1, :, i) = satellites{i}.position;
        histories.force_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i}; 
        histories.magnetic_forces_histories(int32(time/param.dt)+1, :, i) = magnetic_forces{i};
        histories.pair_idx(int32(time/param.dt)+1, i) = i;
    end
end
