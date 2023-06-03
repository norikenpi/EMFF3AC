% 決められた時間シミュレーションを回す
function histories = runSimulation(satellites, param, histories)
    % t秒間のシミュレーションを実行する
    for time = 0:param.dt:param.t
        % 各衛星の状態を更新する
        disp("time")
        disp(time)
        [satellites, histories] = simulateTimeStep(satellites, histories, param, time);
    end
end

