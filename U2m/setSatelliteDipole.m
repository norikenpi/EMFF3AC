% 所望の力から必要な磁気モーメントを計算する
function magnetic_moment = setSatelliteDipole(satellites, u, i, nearest_drift_satellite_idx)
    myu0 = 4*pi*1e-7; % 真空の透磁率

    if isequal(u, [0;0;0])
        satellites{i}.magnetic_moment = [0;0;0];
        magnetic_moment = [0;0;0];
    else
        r = satellites{nearest_drift_satellite_idx}.position - satellites{i}.position;
        r_norm = norm(r);
        %nearest_drift_satellite_idxと所望の力からi番目の衛星の磁気モーメントを設定する。
        m1 = satellites{nearest_drift_satellite_idx}.magnetic_moment;
        if norm(m1) == 0
            m1 = satellites{i}.max_magnetic_moment * r/norm(r);
        end
        D = calculateD(r, m1);
        
        m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u*satellites{i}.mass;
        if norm(m2) > satellites{i}.max_magnetic_moment
            m2 = satellites{i}.max_magnetic_moment/norm(m2) * m2;
        end
        satellites{i}.magnetic_moment = m2;
        magnetic_moment = m2;
    end
end
