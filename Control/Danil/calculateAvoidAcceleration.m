% 衝突回避制御
function u = calculateAvoidAcceleration(param, i, nearest_satellite_idx, satellites)
    r = satellites{i}.position - satellites{nearest_satellite_idx}.position;
    norm_r = norm(r);
    %u = - param.Kp*norm_r^4*[C1, 0, 0].' ;%0.001
    %自分から見た相手の衛星の制御力とは逆の制御力を自分に加える
    u = param.Kp*norm_r^4*(param.safety_distance - norm_r) * r/norm_r;
    %u = param.Kp*norm_r^4*r/norm_r;

end