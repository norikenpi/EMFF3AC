function F_average = satelliteAverageForceTotalonly2(satellite_i, satellite_j, time, param, satellites)
    %interval = 0.001;
    freq_idx = findFrequencyIndex(satellite_i, satellite_j, param);
    if freq_idx == 0
        F_average = zeros(3,1);
    else
        % タイムステップの数
        %シミュレーションのタイムステップよりも短いステップで実際の力を計算してる。
        %t = time:interval:(time + param.dt);
        %satellite_iに関して
        %m_i = satellites{satellite_i}.magnetic_moment(:, freq_idx) * [sin(param.frequency(freq_idx)*t)]; %(3, size(param.frequency_set, 1)) × (size(param.frequency_set, 1)), 1)
        %m_j = satellites{satellite_j}.magnetic_moment(:, freq_idx) * [sin(param.frequency(freq_idx)*t)]; %(3, size(param.frequency_set, 1)) × (size(param.frequency_set, 1)), 1)
        m_i = satellites{satellite_i}.magnetic_moment(:, freq_idx);
        m_j = satellites{satellite_j}.magnetic_moment(:, freq_idx);
    
    
        %mu0 = 4*pi*1e-7; % 真空の透磁率
        r = satellites{satellite_j}.position - satellites{satellite_i}.position;
        %r_norm = norm(r);
        %r_mat = repmat(r, 1, param.dt/interval + 1);
        %if norm(r) > 0.3
        %    A_F = far_field(m_i, m_j, r, param);
        %elseif norm(r) <= 0.3
        %    A_F = near_field(m_i, m_j, r, param);
        %end
        if param.magnetic_model == "near_field"
            if norm(r) < 0.15
                %11分轄
                param.coil_split = 11;
                A_F = near_field(m_i, m_j, r, param);
            elseif norm(r) >= 0.15 && norm(r) < 0.3
                %7分轄
                param.coil_split = 7;
                A_F = near_field(m_i, m_j, r, param);
            elseif norm(r) >= 0.3
                %5分轄
                param.coil_split = 5;
                A_F = near_field(m_i, m_j, r, param);
            end
        elseif param.magnetic_model == "far_field"
            A_F = far_field(m_i, m_j, r, param);
        end
        
        %
        %F = - (3*mu0/(4*pi)) * ((r_mat.*dot(m_i,m_j)/r_norm^5) + (m_i.*dot(m_j,r_mat)/r_norm^5) + (m_j.*dot(m_i,r_mat)/r_norm^5) - r_mat.*(5*dot(m_i,r_mat).*dot(m_j,r_mat))/r_norm^7);
        F_average = A_F/2; %sin^2の積分は1/2
    end
