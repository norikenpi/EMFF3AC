%収束チェック
function [check, histories] = checkConverge(satellites, param, histories, time)


    if param.N == 4
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

    elseif param.N == 8
        border = param.border;
        check = false;
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
    
        histories.relative_distance(int32(time/param.dt)+1, 1) = x1x2;
        histories.relative_distance(int32(time/param.dt)+1, 2) = x2x4;
        histories.relative_distance(int32(time/param.dt)+1, 3) = x3x1;
        histories.relative_distance(int32(time/param.dt)+1, 4) = x3x4;
        histories.relative_distance(int32(time/param.dt)+1, 5) = x5x1;
        histories.relative_distance(int32(time/param.dt)+1, 6) = x6x2;
        histories.relative_distance(int32(time/param.dt)+1, 7) = x7x3;
        histories.relative_distance(int32(time/param.dt)+1, 8) = x8x4;
        if x1x2  < border
            if x2x4 < border
                if x3x4 < border
                    if x3x1  < border
                        if x5x1 < border
                            if x6x2 < border
                                if x7x3 < border
                                    if x8x4 < border
                                        check = true;
                                        disp(x1x2)
                                        disp(x2x4)
                                        disp(x3x4)
                                        disp(x3x1)
                                        disp(x5x1)
                                        disp(x6x2)
                                        disp(x7x3)
                                        disp(x8x4)
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
