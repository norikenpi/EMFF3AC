%(姿勢は考慮できてない)
% 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を求める
function satellites = updateSatelliteStatesString(satellites, param)
    % それぞれの衛星とつながっている衛星との距離を計算する。
    
    for satellite_i = 1:param.N
        for satellite_j = 1:length(param.set)
            r = satellites(satellite_j).position - satellites(satellite_i).position;
            if r > param.satellite_desired_distance
                baseTransformaion(r, satellites(satellite_i).velocity)
            end
        end
    end
end


