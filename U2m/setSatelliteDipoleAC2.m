% 所望の力から必要な磁気モーメントを計算する
% 衛星idxに対応する周波数に所望の制御力を実現する磁気モーメントをわりあてる。
function satellites = setSatelliteDipoleAC2(satellites, u, idx, pair_idx,  histories, time, param)
    myu0 = 4*pi*1e-7; % 真空の透磁率
    


    %satellite_i = param.set_AC(pair_i, 1);
    %satellite_j = param.set_AC(pair_i, 2);
    satellite_i = idx;
    satellite_j = pair_idx;
    
    freq_idx = findFrequencyIndex(idx, pair_idx, param);


    if isequal(u, [0;0;0])
        satellites{satellite_i}.magnetic_moment(:,param.frequency_set(freq_idx)) = [0;0;0];
        satellites{satellite_j}.magnetic_moment(:,param.frequency_set(freq_idx)) = [0;0;0];
        magnetic_moment = [0;0;0];
    else
        %uは衛星群全体で使う周波数の数だけの列を持った3×n行列。4つの衛星の場合3×4行列。
        % pair_satellite_indexはその衛星とペアになる衛星。衛星1なら2と3
        r = satellites{satellite_j}.position - satellites{satellite_i}.position;
        r_norm = norm(r);

        m1 = satellites{satellite_j}.magnetic_moment(:, param.frequency_set(freq_idx));
        if norm(m1) == 0
            m1 = param.coilN * pi * param.radius^2 * param.I_max * r/r_norm;
        end
        D = calculateD(r, m1);
        m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u*satellites{satellite_j}.mass;

        %磁気ダイポールの大きさを均等にする。
        norm_sum = sqrt(norm(m1)*norm(m2));
        
        satellites{satellite_j}.magnetic_moment(:, param.frequency_set(freq_idx)) = norm_sum * m1/norm(m1);
        satellites{satellite_i}.magnetic_moment(:, param.frequency_set(freq_idx)) = norm_sum * m2/norm(m2);

    end
end