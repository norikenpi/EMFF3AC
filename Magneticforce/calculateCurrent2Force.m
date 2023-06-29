function F = calculateCurrent2Force(i1, i2, radius1, radius2, N1, N2, r, param)
    m1 = N1 * i1 * pi * radius1^2;
    m2 = N2 * i2 * pi * radius2^2;
    F = far_field(m1, m2, r, param);