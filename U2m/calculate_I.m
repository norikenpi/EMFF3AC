function Iij = calculate_I(r, m1, i, j, param)
   %コイルjの電流要素単位ベクトルとコイルiが発生する磁場ベクトル×μ0/4πの外積
   %d_l = I2_z * N * rotatepoint(quat2, a * [-sin(phi), cos(phi), 0]) * d_phi;
   Iij = zeros(3,1);
   % m1, m2: 3要素の磁気モーメントベクトル
    % r: 3要素の距離ベクトル (m1からm2までの距離)
    
    
    I1 = m1/(param.coilN * pi * param.radius^2);
    I1_x = I1(1);
    I1_y = I1(2);
    I1_z = I1(3);
    
    quat1 = quaternion([0,0,0],'euler','XYZ','point');
    quat2 = quaternion([0,0,0],'euler','XYZ','point');
    
    x = r(1);
    y = r(2);
    z = r(3);
    a = param.radius;
    N = param.coilN;
    splitA = param.coil_split;
    splitB = param.coil_split;
    d_phi = 2*pi/param.coil_split;
    phi = 0;
    idx = 0;

    if j == 6
        while idx < splitA
            idx = idx + 1;
            phi = phi + d_phi; %電流要素を少しずつずらしてる
            d_l = rotatepoint(quat2, a * [-sin(phi), cos(phi), 0]) * d_phi;
            %i==1のとき，x軸コイルの発生する磁場だけを考えないといけない．
            %B = magnetic_flux_three_coil(rot_point_z(1) + x, rot_point_z(2) + y, rot_point_z(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
            if i == 1
                B = magnetic_flux_x2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_x, a, N, quat1, splitB);
            elseif i == 2
                B = magnetic_flux_y2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_y, a, N, quat1, splitB);
                B = zeros(1,3);
            elseif i == 3
                B = magnetic_flux_z2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_z, a, N, quat1, splitB);
                B = zeros(1,3);
            end
            Iij = Iij + cross(d_l, B).';
        end
    elseif j == 5
        while idx < splitA
            idx = idx + 1;
            phi = phi + d_phi; %電流要素を少しずつずらしてる
            d_l = rotatepoint(quat2, [a * sin(phi), 0, a * cos(phi)]);
            %i==1のとき，x軸コイルの発生する磁場だけを考えないといけない．
            %B = magnetic_flux_three_coil(rot_point_z(1) + x, rot_point_z(2) + y, rot_point_z(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
            if i == 1
                B = magnetic_flux_x2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_x, a, N, quat1, splitB);
            elseif i == 2
                B = magnetic_flux_y2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_y, a, N, quat1, splitB);
                B = zeros(1,3);
            elseif i == 3
                B = magnetic_flux_z2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_z, a, N, quat1, splitB);
                B = zeros(1,3);
            end
            Iij = Iij + cross(d_l, B).';
        end
    elseif j == 4
        while idx < splitA
            idx = idx + 1;
            phi = phi + d_phi; %電流要素を少しずつずらしてる
            d_l = rotatepoint(quat2, [0, a * cos(phi), a * sin(phi)]);
            %i==1のとき，x軸コイルの発生する磁場だけを考えないといけない．
            %B = magnetic_flux_three_coil(rot_point_z(1) + x, rot_point_z(2) + y, rot_point_z(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
            if i == 1
                B = magnetic_flux_x2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_x, a, N, quat1, splitB);
            elseif i == 2
                B = magnetic_flux_y2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_y, a, N, quat1, splitB);
                B = zeros(1,3);
            elseif i == 3
                B = magnetic_flux_z2(d_l(1) + x, d_l(2) + y, d_l(3) + z, I1_z, a, N, quat1, splitB);
                B = zeros(1,3);
            end
            Iij = Iij + cross(d_l, B).';
        end
    end
end