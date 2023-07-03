function F_average = satelliteAverageForceTotal(satellite_i, satellite_j, time, param)
    interval = 0.001;
    % タイムステップの数
    t = time:interval:(time + param.dt);

    %satellite_iに関して
    for i = 1:size(param.frequency_set,1)
        m_i = satellite_i.magnetic_moment * [sin(param.frequency*t)]; %(3, size(param.frequency_set, 1)) × (size(param.frequency_set, 1)), 1)
        m_j = satellite_j.magnetic_moment * [sin(param.frequency*t)]; %(3, size(param.frequency_set, 1)) × (size(param.frequency_set, 1)), 1)
    end



    mu0 = 4*pi*1e-7; % 真空の透磁率
    r = satellite_j.position - satellite_i.position;
    r_norm = norm(r);
    r_mat = repmat(r, 1, param.dt/interval + 1);
    F = - (3*mu0/(4*pi)) * ((r_mat.*dot(m_i,m_j)/r_norm^5) + (m_i.*dot(m_j,r_mat)/r_norm^5) + (m_j.*dot(m_i,r_mat)/r_norm^5) - r_mat.*(5*dot(m_i,r_mat).*dot(m_j,r_mat))/r_norm^7);

    F_average = mean(F, 2);