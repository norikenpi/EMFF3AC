% 衛星の位置、力、および磁気モーメントの履歴を格納するためのセル配列を初期化する
function histories = makeHistoriesMemory(param)
    histories.position_histories = zeros(param.N, int32(param.t/param.dt),3);
    histories.force_histories = zeros(param.N, int32(param.t/param.dt),3);
    histories.magnetic_moment_histories = zeros(param.N, int32(param.t/param.dt),3);
    histories.magnetic_forces_histories = zeros(param.N, int32(param.t/param.dt),3);
    
    
    histories.u_histories = zeros(param.N, int32(param.t/param.dt),3);
    histories.u_real_histories = zeros(param.N, int32(param.t/param.dt),3);
    histories.pair_idx = zeros(param.N, int32(param.t/param.dt));

    %Danilのみ
    %histories.C1_histories = cell(1, param.N);
end

