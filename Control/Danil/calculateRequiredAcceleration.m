% C1を低減するような必要な力を計算する
function u = calculateRequiredAcceleration(param, C1, i, nearest_satellite_idx, satellites)
    norm_r = norm(satellites{i}.position - satellites{nearest_satellite_idx}.position);
    u =  10000*norm_r^4*[C1, 0, 0].' ;%0.001
end