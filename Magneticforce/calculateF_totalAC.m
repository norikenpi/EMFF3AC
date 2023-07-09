function [magnetic_forces, magnetic_torques,histories] = calculateF_totalAC(satellites, param, time, histories)
        % 各衛星に働く磁気トルクと磁気力を計算
    magnetic_torques = cell(1, param.N);
    magnetic_forces = cell(1, param.N);
    for satellite_i = 1:param.N
        magnetic_forces{satellite_i} = zeros(3,1);
        histories.force_histories(int32(time/param.dt)+1, :, satellite_i) = zeros(3,1);
    end


    
    % 1からNまでの衛星に対して、他のすべての衛星の磁気ダイポールによる磁場を計算し、それらの磁場の合計から各衛星に働く磁気トルクと磁気力を求める。
    % 1からNまでの衛星に対して、Far Fieldモデルで電磁力を計算。
    for satellite_i = 1:param.N
        %F_total = zeros(3, 1);
        % それぞれの衛星がある場所での磁力を計算
        %交流の場合の特定の衛星に加わる力を計算する。
        
        %全ての衛星に対して計算
        %{
        for satellite_j = 1:param.N
            
            if satellite_j ~= satellite_i
                F_average_total = satelliteAverageForceTotal(satellites{satellite_i}, satellites{satellite_j}, time, param);
                F_total = F_total + F_average_total;
            end
        end
        %}

        satellite_j = histories.pair_idx(int32(time/param.dt)+1, satellite_i);
        F_average_total = satelliteAverageForceTotal(satellites{satellite_i}, satellites{satellite_j}, time, param);
        % 磁気力を計算

        magnetic_forces{satellite_i} = magnetic_forces{satellite_i} + F_average_total;
        magnetic_forces{satellite_j} = magnetic_forces{satellite_j} - F_average_total;
        histories.force_histories(int32(time/param.dt)+1, :, satellite_i) = histories.force_histories(int32(time/param.dt)+1, :, satellite_i) + (F_average_total).';
        histories.force_histories(int32(time/param.dt)+1, :, satellite_j) = histories.force_histories(int32(time/param.dt)+1, :, satellite_j) - (F_average_total).';
        
    end
end