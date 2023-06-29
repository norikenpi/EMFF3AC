%2つのダイアポール行列から、シミュレーションタイムステップの間にはたらく力を計算
function F_average = satelliteAverageForce(m1_set, m2_set, r, param, time)
    interval = 0.001;
    t = time:interval:(time + param.dt);
    
    sin_mat = zeros(size(param.frequency_set,1), param.dt/interval + 1);

    


    for i = 1:size(param.frequency_set,1)
        sin_mat(i,:) = [sin(param.frequency_set(i)*t)];
    end
    m1 = m1_set * sin_mat;
    m2 = m2_set * sin_mat;    
    mu0 = 4*pi*1e-7; % 真空の透磁率
    r_norm = norm(r);
    r_mat = repmat(r, 1, param.dt/interval + 1);
    F = - (3*mu0/(4*pi)) * ((r_mat.*dot(m1,m2)/r_norm^5) + (m1.*dot(m2,r_mat)/r_norm^5) + (m2.*dot(m1,r_mat)/r_norm^5) - r_mat.*(5*dot(m1,r_mat).*dot(m2,r_mat))/r_norm^7);

    F_average = mean(F, 2);