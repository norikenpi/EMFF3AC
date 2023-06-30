% 衛星の位置、力、および磁気モーメントの履歴を格納するためのセル配列を初期化する
function histories = makeHistoriesMemory(param)
    histories.position_histories = zeros(int32(param.t/param.dt)+1, 3, param.N);
    histories.velocity_histories = zeros(int32(param.t/param.dt)+1, 3, param.N);
    histories.force_histories = zeros(int32(param.t/param.dt), 3, param.N);
    histories.magnetic_moment_histories = zeros(int32(param.t/param.dt)+1, 3, param.N);
    histories.magnetic_moment_req_histories = zeros(int32(param.t/param.dt), 3, param.N);

    histories.magnetic_forces_histories = zeros(int32(param.t/param.dt), 3, param.N);
    
    
    histories.u_histories = zeros(int32(param.t/param.dt), 3, param.N);
    histories.u_real_histories = zeros(int32(param.t/param.dt), 3, param.N);
    histories.pair_idx = zeros(int32(param.t/param.dt)+1, param.N);

    %Danilのみ
    histories.C1_histories = zeros(int32(param.t/param.dt)+1, param.N);
end

