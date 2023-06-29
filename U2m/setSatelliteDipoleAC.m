% 所望の力から必要な磁気モーメントを計算する
function satellites = setSatelliteDipoleAC(satellites, u, pair_i,  histories, time, param)
    myu0 = 4*pi*1e-7; % 真空の透磁率



    satellite_i = param.set_AC(pair_i, 1);
    satellite_j = param.set_AC(pair_i, 2);


    if isequal(u, [0;0;0])
        satellites{satellite_i}.magnetic_moment(:,param.frequency_set(pair_i)) = [0;0;0];
        satellites{satellite_j}.magnetic_moment(:,param.frequency_set(pair_i)) = [0;0;0];
        magnetic_moment = [0;0;0];
    else
        %uは衛星群全体で使う周波数の数だけの列を持った3×n行列。4つの衛星の場合3×4行列。
        % pair_satellite_indexはその衛星とペアになる衛星。衛星1なら2と3
        r = satellites{satellite_j}.position - satellites{satellite_i}.position;
        r_norm = norm(r);
        %freq_idx = findRows(i, satellite_i, param);
        m1 = satellites{satellite_j}.magnetic_moment(:, param.frequency_set(pair_i));
        if norm(m1) == 0
            m1 = param.max_magnetic_moment * r/r_norm;
        end

        D = calculateD(r, m1);
        m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u*satellites{satellite_j}.mass;

        %磁気ダイポールの大きさを均等にする。
        norm_sum = sqrt(norm(m1)*norm(m2));
        satellites{satellite_j}.magnetic_moment(:, param.frequency_set(pair_i)) = norm_sum * m1/norm(m1);
        satellites{satellite_i}.magnetic_moment(:, param.frequency_set(pair_i)) = norm_sum * m2/norm(m2);

    end
end
