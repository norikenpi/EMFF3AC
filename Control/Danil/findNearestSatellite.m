% 与えられた衛星の配列から指定した衛星と除外する衛星以外で、最も近い衛星のインデックスと距離を求めます。
function error_idx = findNearestSatellite(satellites, target_idx, excluded_idx)
    distances = zeros(1, length(satellites));
    error = zeros(1,length(satellites));
    border = 0.30;


    for i = 1:length(satellites)
        if any([i == target_idx, i == excluded_idx, isempty(satellites{i})])
            distances(i) = inf;
            error(i) = 0;
        else
            relative_position = satellites{i}.position - satellites{target_idx}.position;
            relative_position_d = satellites{i}.position_d - satellites{target_idx}.position_d;
            distances(i) = norm(relative_position);
            error(i) = norm(relative_position - relative_position_d);
        end
    end

    %[~, nearest_satellite_idx] = min(distances);
    %[~, error_idx] = max(error);

    % 配列のソート
    sortedArray = sort(error, 'descend'); % 配列を降順にソート
    % ループ処理
    for i = 1:length(sortedArray)
        if sortedArray(i) < border
            %disp('1より小さい数字が見つかりました。');
            error_idx = find(error == sortedArray(i));
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

    
end
