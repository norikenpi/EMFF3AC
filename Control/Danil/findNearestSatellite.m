% 与えられた衛星の配列から指定した衛星と除外する衛星以外で、最も近い衛星のインデックスと距離を求めます。
function nearest_satellite_idx = findNearestSatellite(satellites, target_idx, excluded_idx)
    distances = zeros(1, length(satellites));
    for i = 1:length(satellites)
        if any([i == target_idx, i == excluded_idx, isempty(satellites{i})])
            distances(i) = inf;
        else
            distances(i) = norm(satellites{i}.position - satellites{target_idx}.position);
        end
    end
    [~, nearest_satellite_idx] = min(distances);
end
