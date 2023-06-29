function [u, pair_satellite_list] = controlAlgorithmStepbystep(histories, i, satellites, param, time)


    %ある時刻での衛星iと対応する衛星を設定
    %ペアとなる衛星がいない場合、自衛星と同じインデックスが割り当てられる
    pair_satellite_list = selectSatellitePair(i, time, param);
    
    

    if param.current_type == "DC"
        %該当する衛星だった場合、特定の衛星と相対位置を取ってフィードバック制御して入力を決める。

        if pair_satellite_list ~= i
            pair_satellite_idx = pair_satellite_list;
            u = relativeFeedback(i, pair_satellite_idx, satellites, param);

    
        %該当しない衛星だった場合、入力0
        else
            u = [0;0;0];
        end
    elseif param.current_type == "AC"
        u = zeros(3,size(pair_satellite_list, 2));
        for idx = 1:size(pair_satellite_list, 2)
            pair_satellite_idx = pair_satellite_list(idx);
            u(:, idx) = relativeFeedback(i, pair_satellite_idx, satellites, param);
            disp("u")
            disp(i)
            disp(pair_satellite_idx)
            disp(u)
        end

    end