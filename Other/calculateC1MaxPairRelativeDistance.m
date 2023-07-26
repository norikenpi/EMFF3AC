function distance = calculateC1MaxPairRelativeDistance(satellites, param)
    C1_max = 0;
    i_max = 0;
    for i = 1:param.N
        C1 = abs(satellites{i}.velocity(1) - 2 * param.n * satellites{i}.position(3)); 
        if C1 > C1_max
            C1_max = C1;
            i_max = i;
        end
    end
    max_velocity = 0;
    for i = 1:param.N
        if i ~= i_max
            velocity = norm(satellites{i}.velocity - satellites{i_max}.velocity);
            if velocity > max_velocity
                max_velocity = velocity;
                distance = norm(satellites{i}.position - satellites{i_max}.position);
            end
        end  
    end
end