%m1を発生する磁気モーメントに発生する磁力
function F = far_field(m1, m2, r)

    % m1, m2: 3要素の磁気モーメントベクトル
    % r: 3要素の距離ベクトル (m1からm2までの距離)
    
    % 定数
    mu0 = 4*pi*1e-7; % 真空の透磁率
    % 磁力計算
    r_norm = norm(r);

    if param.current_type == "DC"
        F = - (3*mu0/(4*pi)) * ((r*dot(m1,m2)/r_norm^5) + (m1*dot(m2,r)/r_norm^5) + (m2*dot(m1,r)/r_norm^5) - (5*dot(m1,r)*dot(m2,r)*r/r_norm^7));
    elseif param.current_type == "AC"
       % シミュレーションタイムステップpaaram.dtの間に加わる平均の力を計算
       
        F = - (3*mu0/(4*pi)) * ((r*dot(m1,m2)/r_norm^5) + (m1*dot(m2,r)/r_norm^5) + (m2*dot(m1,r)/r_norm^5) - (5*dot(m1,r)*dot(m2,r)*r/r_norm^7)) * param;
    end
end
