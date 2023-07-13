%発散チェック
function check = checkDiverge(satellites, param)
    border = param.diverge_border;
    check = true;
    x1 = satellites{1}.position;
    x2 = satellites{2}.position;
    x3 = satellites{3}.position;
    x4 = satellites{4}.position;
    if abs(norm(x1 - x2) - param.satellite_desired_distance)  < border
        if abs(norm(x2 - x4) - param.satellite_desired_distance)  < border
            if abs(norm(x3 - x4) - param.satellite_desired_distance)  < border
                if abs(norm(x3 - x1) - param.satellite_desired_distance)  < border
                    check = false;
                end
            end
        end
    end
end
