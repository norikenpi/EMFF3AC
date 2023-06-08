%m1を発生する磁気モーメントに発生する磁力
function F = magneticForce(m1, m2, r, param)
    if param.magnetic_model == "far_field"
        F = far_field(m1, m2, r);
    elseif param.magnetic_model == "near_field"
        F = near_field(m1, m2, r, param);
    end
end
