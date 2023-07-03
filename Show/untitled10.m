%収束チェック
for i = 1:size(histories.position_histories, 1)
    disp("time")
    disp(i)
    x1 = histories.position_histories(i, :, 1);
    x2 = histories.position_histories(i, :, 2);
    x3 = histories.position_histories(i, :, 3);
    x4 = histories.position_histories(i, :, 4);
    if abs(norm(x1 - x2) - 0.15)  < 0.005
        if abs(norm(x2 - x4) - 0.15)  < 0.005
            if abs(norm(x3 - x4) - 0.15)  < 0.005
                if abs(norm(x3 - x1) - 0.15)  < 0.005
                    disp("converge")
                    disp(i)
                    break
                end
            end
        end
    end
    
end
disp(abs(norm(x1 - x2) - 0.15))
disp(abs(norm(x2 - x4) - 0.15))
disp(abs(norm(x3 - x4) - 0.15))
disp(abs(norm(x3 - x1) - 0.15))
