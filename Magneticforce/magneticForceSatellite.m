function F = magneticForceSatellite(satellite_i, satellite_j, satellites, param)
    r = satellites{satellite_j}.position - satellites{satellite_i}.position;
    m1 = satellites{satellite_i}.magnetic_moment;
    m2 = satellites{satellite_j}.magnetic_moment;
    F = magneticForce(m1, m2, r, param);
end

