% 与えられた衛星の配列から指定した衛星と除外する衛星以外で、最も近い衛星のインデックスと距離を求めます。
function [error_idx, histories, safety_distance, satellites] = findNearestSatellite(satellites, target_idx, excluded_idx, param, histories, time)
    distances = zeros(1, length(satellites));
    error = zeros(1,length(satellites));
    safety_distance = 0;
    C1 = 0;


    for i = 1:length(satellites)
        %自分と同じインデックスの場合
        if any([i == target_idx, i == excluded_idx, isempty(satellites{i})])
            distances(i) = 1/100000;
            error(i) = 0;
            
        else
            relative_position =  satellites{target_idx}.position - satellites{i}.position;
            if norm(relative_position) < param.diverge_border
                relative_position_d = satellites{target_idx}.position_d - satellites{i}.position_d;
                distances(i) = 1/norm(relative_position);
    
                relative_velocity =  satellites{target_idx}.velocity - satellites{i}.velocity;
                target_error = norm(relative_position_d - relative_position);
                C1 = relative_velocity(1)/param.n - 2 * relative_position(3); 
                C4 = relative_position(1) + 2 * relative_velocity(3)/param.n;
                histories.C1_histories(int32(time/param.dt)+1, i) = C1;
                satellites{i}.C1 = C1;
    
                %最も近くにある衛星が衝突防止制御する範囲にある場合
                if (1/distances(i)) < param.safety_distance
                   safety_distance = 1;
                else
                    %提案アルゴリズム別のペア決め方法
                    if param.pair_type == "C1"
                        %error(i) = norm(relative_position - relative_position_d); %目標相対位置誤差
                        error(i) = C1;
                    elseif param.pair_type == "distance"
                        %distances(i) = norm(relative_position);
                        error(i) = 1/norm(relative_position);
                    elseif param.pair_type == "target_distance" %相対目標位置誤差
                        error(i) = target_error;
                    elseif param.pair_type == "velocity"
                        error(i) = norm(relative_velocity);
                    elseif param.pair_type == "all_energy"
                        error(i) = (norm(relative_velocity)^2)*param.mass/2 + (param.n*relative_position(2)^2)/2 - (3*param.n*relative_position(3)^2)/2;
                    elseif param.pair_type == "separate_velocity"
                        if  dot(relative_velocity, relative_position) > 0
                            error(i) = norm(relative_velocity)*100;
                        else
                            error(i) = norm(relative_velocity);
                        end
                    elseif param.pair_type == "Takahashi"
                        rs = 10;
                        if (C4^2 + (2*C1)^2 < rs) || (C1*C4 > 0)
                            error(i) = 1/((C4 + sign(C1) * sqrt(rs^2 - (2 * C1)^2))/(2*C1*param.n));
                        else
                            error(i) = 0;
                        end
                    end
                end
            end
        end
    end
    abs_error = abs(error);
    
    if safety_distance == 1
        abs_error = distances;
    end
    %[~, nearest_satellite_idx] = min(distances);
    %[~, error_idx] = max(error);

    % 配列のソート
    sortedArray = sort(abs_error, 'descend'); % 配列を降順にソート
    % ループ処理
    border = 5000000000;
    for i = 1:length(sortedArray)
        idx = find(abs_error == sortedArray(i));
        %ボーダーあり
        if abs_error(idx) < border
            %disp('1より小さい数字が見つかりました。');
            error_idx = idx(1);
            histories.relative_distance_pair(int32(time/param.dt)+1, target_idx) = 1/distances(error_idx );
            %satellites{i}.C1 = error(idx);
            break; % ループを終了
        
        end
        
        
    end

    %道グラフ
    %{
    if target_idx == 1
        error_idx = 2;
    elseif target_idx == 2
        error_idx = 3;
    elseif target_idx == 3
        error_idx = 2;
    elseif target_idx == 4
        error_idx = 3;
    end
    %}
    %disp(error_idx)

    
end
