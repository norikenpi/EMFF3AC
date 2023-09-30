function satellites = adjustMagneticMomentAC(satellites, param)
    
    for satellite_i = 1:param.N
        magnetic_moment_norm_list = vecnorm(satellites{satellite_i}.magnetic_moment);
        magnetic_moment_norm = sum(magnetic_moment_norm_list);
        %disp("disp")
        %disp(magnetic_moment_norm)
        %disp(param.max_magnetic_moment)
        if magnetic_moment_norm > param.max_magnetic_moment
            
            %disp("overover")
            satellites{satellite_i}.magnetic_moment = satellites{satellite_i}.magnetic_moment*param.max_magnetic_moment/magnetic_moment_norm;
        end
    end