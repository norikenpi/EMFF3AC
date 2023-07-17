function m2 = far_field_inv(r, m1, u, param)
    myu0 = 4*pi*1e-7; % 真空の透磁率
    r_norm = norm(r);
    D = calculateD(r, m1);
    m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u*param.mass;
end