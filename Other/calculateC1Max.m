function C1_max = calculateC1Max(satellites, param)
    C1_max = 0;
    for i = 1:param.N
        C1 = abs(satellites{i}.velocity(1) - 2 * param.n * satellites{i}.position(3)); 
        if C1 > C1_max
            C1_max = C1;
        end
    end
end