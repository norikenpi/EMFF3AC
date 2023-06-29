function h = showCoilDipoleError2(distance, coil_radius, param)

    %電流要素分割数と衛星間距離とコイル半径を変更して力とトルクの精度を確かめる．
    addpath("C:/Users/masam/lab/30_simscape/20_磁石/EMFF3/Magneticforce");
    a = coil_radius;
    N = 17;
    i_max = 6;
    i = 0;
    splitA = 5;
    splitB = 5;
    data = zeros(i_max-2, 4);
    X = [distance 0 0];
    I1 = [1 0 0];
    I2 = [1 0 0];
    p1 = 0;
    q1 = 0;
    l1 = 0;
    p2 = 0;
    q2 = 0;
    l2 = 0;
    quat1 = quaternion([p1, q1, l1],'euler','XYZ','point');
    quat2 = quaternion([p2, q2, l2],'euler','XYZ','point');
    
    
    m1 = pi*a^2*N*[1;0;0];
    m2 = pi*a^2*N*[1;0;0];
    r = [0.15;0;0];
    F_dipole = far_field(m1, m2, r, param);
    disp("ダイポール近似")
    disp(F_dipole)
    disp(norm(F_dipole))
    
    [F100, T100] = plot_magnetic_field_FT2(quat1, quat2, X, I1, I2, 100, 100, N, a);
    n_F100 = norm(F100);
    n_T100 = norm(T100);
    disp("100分割")
    disp(F100)
    disp(n_F100)
    
    
    
    disp("誤差")
    disp(norm(F_dipole)/n_F100 - 1)
    
    while i < i_max
        disp(i)
        i = i + 1;
        F = magneticForce(m1, m2, r, param, 1);
        data(i,:) = [splitB, splitA, (norm(F)/n_F100-1) * 100, (norm(F)/n_T100-1) * 100];
    end
    
    %data = [data; [100, 100, ％F, T]];
    
    %電流要素分割数に対する計算精度の検証のためのグラフ
    

    
    h = plot(data(:,1), data(:,3));
    
    %grid on
    %xlabel('電流要素分割数')
    %ylabel('100分割に対する誤差(%)')
    %title('電流要素分割数に対する力の精度')
    %legend(string(distance))
    %hold on

    
    
    %{
    figure 
    
    plot(data(:,1), data(:,4))
    hold on
    grid on
    xlabel('電流要素分割数')
    ylabel('トルク(N・m)')
    title('電流要素分割数に対するトルクの精度')
    %}