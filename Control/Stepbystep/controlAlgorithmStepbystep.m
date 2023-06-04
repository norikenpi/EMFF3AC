function [u, pair_satellite_idx] = controlAlgorithmStepbystep(histories, i, satellites, param, time)


    %ペアとなる衛星を決定する。
    %ペアとなる衛星がいない場合、自衛星と同じインデックスが割り当てられる
    pair_satellite_idx = selectSatellitePair(i, time, param);

    %該当する衛星だった場合、特定の衛星と相対位置を取ってフィードバック制御して入力を決める。
    if pair_satellite_idx ~= i
        u = relativeFeedback(i, pair_satellite_idx, satellites, param);

    %該当しない衛星だった場合、入力0
    else
        u = [0;0;0];
    end