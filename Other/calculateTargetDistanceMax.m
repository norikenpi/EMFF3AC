function target_distace_max = calculateTargetDistanceMax(satellites, param)
    target_distace_max = 0;
    for i = 1:param.N
        target_distace = norm(satellites{i}.position -satellites{i}.position_d);
        if target_distace > target_distace_max
            target_distace_max = target_distace;
        end
    end
end