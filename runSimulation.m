% 決められた時間シミュレーションを回す
function [satellites, histories] = runSimulation(satellites, param, histories)
    % t秒間のシミュレーションを実行する
    tic;
    for time = 0:param.dt:param.t
        % 各衛星の状態を更新する
        disp(time)
        if param.current_type == "DC"
            [satellites, histories] = simulateTimeStep(satellites, histories, param, time);
        elseif param.current_type == "AC"
            [satellites, histories] = simulateTimeStepAC(satellites, histories, param, time);
            %[satellites, histories] = simulateTimeStepACAll(satellites, histories, param, time);
        end
    end
    toc;
end

