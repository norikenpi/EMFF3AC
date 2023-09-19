function [nearest_idx, distance_min] = findNearestDistanceSatellite(satellites, idx, param)
    distance_min = 1000;
    nearest_idx = 0;
    for i = 1:param.N
        if i ~= idx
            distance = norm(satellites{idx}.position - satellites{i}.position);
            if distance_min > distance
                distance_min = distance;
                nearest_idx = i;
            end
        end
    end
end