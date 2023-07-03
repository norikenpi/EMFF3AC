function [u, nearest_satellite_idx] = controlAlgorithmDanilnotC1(histories, i, satellites, param, time)
    nearest_satellite_idx = findNearestSatellite(satellites, i, i);

    u = relativeFeedback(i, nearest_satellite_idx, satellites, param);

end