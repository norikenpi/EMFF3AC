%原点にあるz方向のN巻き，半径a，電流Iのコイルによって，(x, y, z)にあるN巻き，半径a，電流I_x, I_y, I_zのコイルに発生するアンペール力（力，トルク）
function F = zcoil2xyzcoil(m1, m2, r, param)

    % m1, m2: 3要素の磁気モーメントベクトル
    % r: 3要素の距離ベクトル (m1からm2までの距離)
    
    I1 = m1/(param.coilN * pi * param.radius^2);
    I2 = m2/(param.coilN * pi * param.radius^2);
    I1_x = I1(1);
    I1_y = I1(2);
    I1_z = I1(3);
    I2_x = I2(1);
    I2_y = I2(2);
    I2_z = I2(3);

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
    i = 0;
    F = zeros(1,3);

    %z coil
    while i < splitA
        i = i + 1;
        phi = phi + d_phi;
        rot_point_z = rotatepoint(quat2, [a * cos(phi), a * sin(phi), 0]);
        B = magnetic_flux_three_coil(rot_point_z(1) + x, rot_point_z(2) + y, rot_point_z(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
        d_I = I2_z * N * rotatepoint(quat2, a * [-sin(phi), cos(phi), 0]) * d_phi;
        d_F = cross(d_I, B);
        %d_T = cross(rotatepoint(quat2, [a*cos(phi), a*sin(phi), 0]), d_F);
        F = F + d_F;
    end

    %y_coil
    i = 0;
    phi = 0;
    while i < splitA
        
        i = i + 1;
        phi = phi + d_phi;
        rot_point_y = rotatepoint(quat2, [a * sin(phi), 0, a * cos(phi)]);
        B = magnetic_flux_three_coil(rot_point_y(1) + x, rot_point_y(2) + y, rot_point_y(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB);
        d_I = I2_y  * N* rotatepoint(quat2, a * [cos(phi), 0, -sin(phi)] )* d_phi;
        
        d_F = cross(d_I, B);
        %d_F = Ampere(I, phi, B);
        %d_T = cross(rotatepoint(quat2, [a*sin(phi), 0, a*cos(phi)]), d_F);
        %disp(d_B)
        F = F + d_F;
        %T = T + d_T;
        
        
    
    end
    
    %x_coil
    i = 0;
    phi = 0;
    while i < splitA
        i = i + 1;
        phi = phi + d_phi;
        rot_point_x = rotatepoint(quat2, [0, a * cos(phi), a * sin(phi)]);
        B = magnetic_flux_three_coil(rot_point_x(1) + x, rot_point_x(2) + y, rot_point_x(3) + z, I1_x, I1_y, I1_z, a, N, quat1, splitB); 
        d_I = I2_x * N* rotatepoint(quat2, a * [0, -sin(phi), cos(phi)]) * d_phi;
        %d_I = I2_x * N* [0, -sin(phi), cos(phi)]  * d_phi;
        d_F = cross(d_I, B);
        %d_F = Ampere(I, phi, B);
        %disp(d_F)
        %d_T = cross(rotatepoint(quat2, [0, a*cos(phi), a*sin(phi)]), d_F);
        %disp(d_B)
        F = F + d_F;
        %disp("d_F is")
        %disp(d_F)
        %disp("F is")
        %disp(F)
        %T = T + d_T;
    end
    F = F.';
end