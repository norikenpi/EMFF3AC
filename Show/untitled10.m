%収束チェック
border = param.border;
for i = 1:size(histories.position_histories, 1)
    disp("time")
    disp(i)
    x1 = histories.position_histories(i, :, 1);
    x2 = histories.position_histories(i, :, 2);
    x3 = histories.position_histories(i, :, 3);
    x4 = histories.position_histories(i, :, 4);
    if abs(norm(x1 - x2) - param.satellite_desired_distance)  < border
        if abs(norm(x2 - x4) - param.satellite_desired_distance)  < border
            if abs(norm(x3 - x4) - param.satellite_desired_distance)  < border
                if abs(norm(x3 - x1) - param.satellite_desired_distance)  < border
                    disp("converge")
                    disp(i)
                    break
                end
            end
        end
    end

end
disp(abs(norm(x1 - x2) - param.satellite_desired_distance))
disp(abs(norm(x2 - x4) - param.satellite_desired_distance))
disp(abs(norm(x3 - x4) - param.satellite_desired_distance))
disp(abs(norm(x3 - x1) - param.satellite_desired_distance))
