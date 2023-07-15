% 決められた時間シミュレーションを回す
function [satellites, histories, param] = runSimulation(satellites, param, histories)
    % t秒間のシミュレーションを実行する
    tic;
    for time = 0:param.dt:param.t
        % 各衛星の状態を更新する
        %disp(time)
        if param.current_type == "DC"
            [satellites, histories] = simulateTimeStep(satellites, histories, param, time);
        elseif param.current_type == "AC"
            [satellites, histories] = simulateTimeStepAC(satellites, histories, param, time);
            %[satellites, histories] = simulateTimeStepACAll(satellites, histories, param, time);
        end

        %収束または発散のチェックを行う。
        if mod(time, param.time_step) == 0
            [converge_check, histories] = checkConverge(satellites, param, histories, time);
            diverge_check = checkDiverge(satellites, param);
            if converge_check
                param.finished_time = time;
                disp("converge")
                disp(param.finished_time)
                break
            elseif diverge_check
                param.finished_time = -time;
                disp("diverge")
                disp(param.finished_time)
                break
            
            end
        end
    end
    toc;
end

