function satellites = adjustMagneticMoment(satellites, param)
    for satellite_i = 1:param.N
        magnetic_moment_sum = sum(satellites{satellite_i}.magnetic_moment, 2);
        magnetic_moment_norm = norm(magnetic_moment_sum);
        if magnetic_moment_norm > param.max_magnetic_moment
            satellites{satellite_i}.magnetic_moment = satellites{satellite_i}.magnetic_moment*param.max_magnetic_moment/magnetic_moment_norm;
        end
    end