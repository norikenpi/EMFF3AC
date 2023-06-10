% 決められた時間シミュレーションを回す
function [satellites, histories] = runSimulation(satellites, param, histories)
    % t秒間のシミュレーションを実行する
    tic;
    for time = 0:param.dt:param.t
        % 各衛星の状態を更新する
        disp("time")
        disp(time)
        [satellites, histories] = simulateTimeStep(satellites, histories, param, time);
    end
    toc;
end

