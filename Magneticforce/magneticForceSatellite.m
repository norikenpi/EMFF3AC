function F = magneticForceSatellite(satellite_i, satellite_j, satellites, param)
    if satellite_i == satellite_j
        F = zeros(3,1);
    else

        r = satellites{satellite_j}.position - satellites{satellite_i}.position;
        m1 = satellites{satellite_i}.magnetic_moment;
        m2 = satellites{satellite_j}.magnetic_moment;
        disp(m2)
        F = magneticForce(m1, m2, r, param);
    end
end

