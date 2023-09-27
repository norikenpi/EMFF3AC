%収束チェック
function check = checkConverge2(satellites, param, histories, time)
    for i = 1:param.N
        check = true;
        [nearest_idx, distance] = findNearestDistanceSatellite(satellites, i, param);
        relative_target_distance = abs(distance - param.satellite_desired_distance);
        histories.relative_target_distance(int32(time/param.dt)+1, i) = relative_target_distance;
        if relative_target_distance > param.border % もし収束borderよりも大きかった場合、false
            check = false;
            break
        end
    end
end