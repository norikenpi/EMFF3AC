%発散チェック
function check = checkDiverge(satellites, param)
    

    if param.N == 4
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
    elseif param.N == 8
        border = param.diverge_border;
        check =true;
        x1 = satellites{1}.position;
        x2 = satellites{2}.position;
        x3 = satellites{3}.position;
        x4 = satellites{4}.position;
        x5 = satellites{5}.position;
        x6 = satellites{6}.position;
        x7 = satellites{7}.position;
        x8 = satellites{8}.position;
        x1x2 = abs(norm(x1 - x2) - param.satellite_desired_distance);
        x2x4 = abs(norm(x2 - x4) - param.satellite_desired_distance);
        x3x4 = abs(norm(x3 - x4) - param.satellite_desired_distance);
        x3x1 = abs(norm(x3 - x1) - param.satellite_desired_distance);
        x5x1 = abs(norm(x5 - x1) - param.satellite_desired_distance);
        x6x2 = abs(norm(x6 - x2) - param.satellite_desired_distance);
        x7x3 = abs(norm(x7 - x3) - param.satellite_desired_distance);
        x8x4 = abs(norm(x8 - x4) - param.satellite_desired_distance);
        if x1x2  < border
            if x2x4 < border
                if x3x4 < border
                    if x3x1  < border
                        if x5x1 < border
                            if x6x2 < border
                                if x7x3 < border
                                    if x8x4 < border
                                        check = false;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    elseif param.N == 3
        border = param.diverge_border;
        check = true;
        x1 = satellites{1}.position;
        x2 = satellites{2}.position;
        x3 = satellites{3}.position;
        if abs(norm(x1 - x2) - param.satellite_desired_distance)  < border
            if abs(norm(x2 - x3) - param.satellite_desired_distance)  < border
                if abs(norm(x3 - x1) - param.satellite_desired_distance)  < border
                    check = false;
                end
            end
        end
    end
end
