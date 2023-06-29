%m1を発生する磁気モーメントに発生する磁力
%2つの衛星間で発生する磁力
function F = magneticForce(m1, m2, r, param, time)

    if param.current_type == "DC"
        if param.magnetic_model == "far_field"
            F = far_field(m1, m2, r, param);
        elseif param.magnetic_model == "near_field"
            F = near_field(m1, m2, r, param);
        end
    elseif param.current_type == "AC"
        %三角関数の加瀬根合わせ関数を掛け合わせたものの平均値を使って特定の衛星間の磁力を計算
        %m1は各周波数のベクトルを合わせたもの。
        F = satelliteAverageForce(m1, m2, r, param, time);
    end
end
