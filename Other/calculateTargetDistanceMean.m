function target_distace_mean = calculateTargetDistanceMean(satellites, param)
    target_distace_mean = 0;
    for i = 1:param.N
        target_distace_mean = target_distace_mean + norm(satellites{i}.position -satellites{i}.position_d);
    end
end