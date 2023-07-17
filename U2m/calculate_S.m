function S = calculate_S(r, m1, param)
%行列Sの1列目はI14,I24,I34を足したもの．つまり，コイル4に単位電流が流れるときにコイル4に加わる力のこと．
    I1 = m1/(param.coilN * pi * param.radius^2);
    I1_x = param.coilN * I1(1);
    I1_y = param.coilN * I1(2);
    I1_z = param.coilN * I1(3);

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
    I4 = 0;
    I5 = 0;
    I6 = 0;
    %I2のz coilに働く磁力
    i = 0;
    phi = 0;
    while i < splitA
        i = i + 1;
        phi = phi + d_phi; %電流要素を少しずつずらしてる．
        rot_point_z = rotatepoint(quat2, [a * cos(phi), a * sin(phi), 0]);
        %その電流要素での磁場を計算
        B = magnetic_flux_three_coil(rot_point_z(1) + x, rot_point_z(2) + y, rot_point_z(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
        %電流要素ベクトルを計算
        d_I = rotatepoint(quat2, a * [-sin(phi), cos(phi), 0]) * d_phi;
        d_F = cross(d_I, B);
        I6 = I6 + d_F;
    end
    %y_coil
    i = 0;
    phi = 0;
    while i < splitA
        i = i + 1;
        phi = phi + d_phi;
        rot_point_y = rotatepoint(quat2, [a * sin(phi), 0, a * cos(phi)]);
        B = magnetic_flux_three_coil(rot_point_y(1) + x, rot_point_y(2) + y, rot_point_y(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
        d_I = rotatepoint(quat2, a * [cos(phi), 0, -sin(phi)] )* d_phi;
        d_F = cross(d_I, B);
        I5 = I5 + d_F;   
    end
    %x_coil
    i = 0;
    phi = 0;
    while i < splitA
        i = i + 1;
        phi = phi + d_phi;
        rot_point_x = rotatepoint(quat2, [0, a * cos(phi), a * sin(phi)]);
        B = magnetic_flux_three_coil(rot_point_x(1) + x, rot_point_x(2) + y, rot_point_x(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB); 
        d_I = rotatepoint(quat2, a * [0, -sin(phi), cos(phi)]) * d_phi;
        d_F = cross(d_I, B);
        I4 = I4 + d_F;
    end

    S = [I4;I5;I6].';
end