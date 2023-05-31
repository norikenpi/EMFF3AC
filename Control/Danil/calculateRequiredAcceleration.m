% C1を低減するような必要な力を計算する
function u = calculateRequiredAcceleration(param, C1)
    u = -param.Kp*[C1, 0, 0].';%0.001
end