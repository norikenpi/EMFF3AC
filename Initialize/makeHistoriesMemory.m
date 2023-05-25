% 衛星の位置、力、および磁気モーメントの履歴を格納するためのセル配列を初期化する
function histories = makeHistoriesMemory(param)
    histories.position_histories = cell(1, param.N);
    histories.force_histories = cell(1, param.N);
    histories.magnetic_moment_histories = cell(1, param.N);
    histories.C1_histories = cell(1, param.N);
    histories.magnetic_forces_histories = cell(1, param.N);
    histories.u_histories = cell(1, param.N);
    histories.u_real_histories = cell(1, param.N);
    histories.pair_idx = cell(1, param.N);
end

