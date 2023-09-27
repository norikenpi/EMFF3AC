% 決められた時間シミュレーションを回す
function [satellites, histories, param] = runSimulation(satellites, param, histories)
    % t秒間のシミュレーションを実行する

    for time = 0:param.dt:param.t
        % 各衛星の状態を更新する
        disp(time)
        if param.current_type == "DC"
            [satellites, histories] = simulateTimeStep(satellites, histories, param, time);
        elseif param.current_type == "AC"
            %startTime = datetime;
        
            [satellites, histories, param] = simulateTimeStepAC(satellites, histories, param, time);

            %endTime = datetime;
            %executionTime = endTime - startTime;
            %timeString = [num2str(milliseconds(executionTime), '%.2f') ' ms'];
            %disp(['simulateTimeStepAC処理時間: ' timeString]);



            %[satellites, histories] = simulateTimeStepACAll(satellites, histories, param, time);
        end

        %収束または発散のチェックを行う。
        if param.converge_check
            if mod(time, param.time_step) == 0
                %[converge_check, histories] = checkConverge(satellites, param, histories, time);
                converge_check = checkConverge2(satellites, param, histories, time); %これ最も近い衛星とペア組んでるけどダメだわ。
                %diverge_check = checkDiverge(satellites, param);
                diverge_check = checkDiverge2(satellites, param, histories, time);
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
        
    end
    
end

