function m2 = near_field_inv(r, m1, u, param)
    myu0 = 4*pi*1e-7; % 真空の透磁率
    S = calculate_S(r, m1, param);
    %m2 = 4*pi/myu0 * inv(S) * u * param.mass;
    m2 = - param.coilN * pi * param.radius^2 * inv(S) * u * param.mass;
end