function distance_mean = calculateDistanceMean(satellites, param)
    distance_mean = 0;
    for i = 1:param.N
        distance_mean = distance_mean + norm(satellites{i}.position);
    end
end