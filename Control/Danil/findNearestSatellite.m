% 与えられた衛星の配列から指定した衛星と除外する衛星以外で、最も近い衛星のインデックスと距離を求めます。
function [error_idx, C1] = findNearestSatellite(satellites, target_idx, excluded_idx, param)
    distances = zeros(1, length(satellites));
    error = zeros(1,length(satellites));
    C1 = 0;
    


    for i = 1:length(satellites)
        if any([i == target_idx, i == excluded_idx, isempty(satellites{i})])
            distances(i) = inf;
            error(i) = 0;
        else
            relative_position =  satellites{target_idx}.position - satellites{i}.position;
            relative_velocity =  satellites{target_idx}.velocity - satellites{i}.velocity;
            %relative_position_d = satellites{i}.position_d - satellites{target_idx}.position_d;
            C1 = relative_velocity(1) - 2 * param.n * relative_position(3);
            distances(i) = norm(relative_position);
            %error(i) = norm(relative_position - relative_position_d); %目標相対位置誤差
            error(i) = C1;
        end
    end
    abs_error = abs(error);

    %[~, nearest_satellite_idx] = min(distances);
    %[~, error_idx] = max(error);

    % 配列のソート
    sortedArray = sort(abs_error, 'descend'); % 配列を降順にソート
    % ループ処理
    border = 50;
    for i = 1:length(sortedArray)
        idx = find(abs_error == sortedArray(i));
        %ボーダーあり
        if abs_error < border
            %disp('1より小さい数字が見つかりました。');
            
            error_idx = idx;
            C1 = error(idx);
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
