function energy_sum = calculateEnergySum(satellites, param)
    energy_sum = 0;
    for i = 1:param.N
        energy_sum = energy_sum + abs((norm(satellites{i}.velocity)^2)/2 + (param.n*satellites{i}.position(2)^2)/2 - (3*param.n*satellites{i}.position(3)^2)/2);
    
    end
        
end