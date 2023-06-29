% 所望の力から必要な磁気モーメントを計算する
function [magnetic_moment, magnetic_moment_req] = setSatelliteDipole(satellites, u, i, pair_satellite_idx_list, histories, time, param)
    myu0 = 4*pi*1e-7; % 真空の透磁率

    if isequal(u, [0;0;0])
        satellites{i}.magnetic_moment = [0;0;0];
        magnetic_moment = [0;0;0];
        magnetic_moment_req = magnetic_moment;
    else
        if param.current_type == "DC"
            pair_satellite_idx = pair_satellite_idx_list;
            r = satellites{pair_satellite_idx}.position - satellites{i}.position;
            r_norm = norm(r);
            %nearest_drift_satellite_idxと所望の力からi番目の衛星の磁気モーメントを設定する。
            m1 = satellites{pair_satellite_idx}.magnetic_moment;
            if norm(m1) == 0
                m1 = satellites{i}.max_magnetic_moment * r/r_norm;
                satellites{i}.magnetic_moment = m1;
            end
            D = calculateD(r, m1);
            m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u*satellites{i}.mass;
            magnetic_moment_req = m2;
            if norm(m2) > satellites{i}.max_magnetic_moment
                m2 = satellites{i}.max_magnetic_moment/norm(m2) * m2;
            end
            satellites{i}.magnetic_moment = m2;
            magnetic_moment = m2;



        elseif param.current_type == "AC"
            %uは衛星群全体で使う周波数の数だけの列を持った3×n行列。4つの衛星の場合3×4行列。
            % pair_satellite_indexはその衛星とペアになる衛星。衛星1なら2と3
            for idx = 1:size(pair_satellite_idx_list, 2)
                pair_satellite = pair_satellite_idx_list(idx);
                r = satellites{pair_satellite}.position - satellites{i}.position;
                r_norm = norm(r);
                freq_idx = findRows(i, pair_satellite, param);
                m1 = satellites{pair_satellite}.magnetic_moment(:, freq_idx);
                if norm(m1) == 0
                    m1 = (satellites{pair_satellite}.max_magnetic_moment/size(param.set{pair_satellite}, 2)) * r/r_norm;
                    satellites{pair_satellite}.magnetic_moment(:, freq_idx) = m1;
                    disp("if 0")
                    disp(pair_satellite)
                    disp(satellites{pair_satellite}.magnetic_moment)
                    
                end

                D = calculateD(r, m1);
                m2 = - 4*pi*r_norm^5/(3*myu0)*inv(D)*u(:, idx)*satellites{i}.mass;

                magnetic_moment_req = m2;
                if norm(m2) > satellites{i}.max_magnetic_moment
                    m2 = (satellites{i}.max_magnetic_moment/size(param.set{i}, 2)) * m2/norm(m2);
                end
                
                
                satellites{i}.magnetic_moment(:, freq_idx) = m2;
                disp("mag_mo i")

                disp(i)
                disp(pair_satellite)
                disp("u")
                disp(u)
                disp(satellites{i}.magnetic_moment)
                disp(satellites{pair_satellite}.magnetic_moment)
                magnetic_moment = m2;

            end
        end


    end
end
