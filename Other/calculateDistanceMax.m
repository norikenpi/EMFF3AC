function distance_max = calculateDistanceMax(satellites, param)
    distance_max = 0;
    for i = 1:param.N
        distance = norm(satellites{i}.position);
        if distance > distance_max
            distance_max = distance;
        end
    end
end