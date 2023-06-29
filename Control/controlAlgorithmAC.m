function u = controlAlgorithmAC(histories, pair, satellites, param, time)        
    %ある時刻での衛星iと対応する衛星を設定
    %ペアとなる衛星がいない場合、自衛星と同じインデックスが割り当てられる
    satellite_i = pair(1);
    satellite_j = pair(2);
    u = relativeFeedback(satellite_i, satellite_j, satellites, param);