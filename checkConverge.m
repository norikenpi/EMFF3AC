%収束チェック
function [check, histories] = checkConverge(satellites, param, histories, time)
    border = param.border;
    check = false;
    x1 = satellites{1}.position;
    x2 = satellites{2}.position;
    x3 = satellites{3}.position;
    x4 = satellites{4}.position;
    x1x2 = abs(norm(x1 - x2) - param.satellite_desired_distance);
    x2x4 = abs(norm(x2 - x4) - param.satellite_desired_distance);
    x3x4 = abs(norm(x3 - x4) - param.satellite_desired_distance);
    x3x1 = abs(norm(x3 - x1) - param.satellite_desired_distance);

    histories.relative_distance(int32(time/param.dt)+1, 1) = x1x2;
    histories.relative_distance(int32(time/param.dt)+1, 2) = x2x4;
    histories.relative_distance(int32(time/param.dt)+1, 3) = x3x4;
    histories.relative_distance(int32(time/param.dt)+1, 4) = x3x1;
    if x1x2  < border
        if x2x4 < border
            if x3x4 < border
                if x3x1  < border
                    check = true;
                    disp(x1x2)
                    disp(x2x4)
                    disp(x3x4)
                    disp(x3x1)
                end
            end
        end
    end
end
