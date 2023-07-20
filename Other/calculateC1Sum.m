function C1_sum = calculateC1Sum(satellites, param)
    C1_sum = 0;
    for i = 1:param.N
        C1_sum = C1_sum + abs(satellites{i}.velocity(1) - 2 * param.n * satellites{i}.position(3)); 
    end
end