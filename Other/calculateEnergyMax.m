function energy_max = calculateEnergyMax(satellites, param)
    energy_max = 0;
    for i = 1:param.N
        energy = abs((norm(satellites{i}.velocity)^2)/2 + (param.n*satellites{i}.position(2)^2)/2 - (3*param.n*satellites{i}.position(3)^2)/2);
        if energy > energy_max
            energy_max = energy;
        end
    
    end
        
end