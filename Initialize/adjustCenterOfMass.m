function adjustCenterOfMass(satellites, param)


%初期値の重心位置と重心速度を計算
    CoM = zeros(3,1);
    CoMV = zeros(3,1);

    %重心位置と重心速度を計算
    for i = 1:param.N
        CoM = CoM + satellites{i}.position/param.N;
        CoMV = CoMV + satellites{i}.velocity/param.N;
        disp('CoM')
        disp(CoM)
        disp('CoMV')
        disp(CoMV)
    end
    
    %重心位置と重心速度を0にするように設定
    for i = 1:param.N
        satellites{i}.position = satellites{i}.position - CoM;
        satellites{i}.velocity = satellites{i}.velocity - CoMV;
        disp("修正済み")
        disp(satellites{i}.position)
        disp(satellites{i}.velocity)

    end


%目標値の重心位置と重心速度を計算
    CoM = zeros(3,1);
    CoMV = zeros(3,1);
    for i = 1:param.N
        CoM = CoM + satellites{i}.position_d/param.N;
        CoMV = CoMV + satellites{i}.velocity_d/param.N;
        disp('CoM')
        disp(CoM)
        disp('CoMV')
        disp(CoMV)
    end
    
    %重心位置と重心速度を0にするように設定
    for i = 1:param.N
        satellites{i}.position_d = satellites{i}.position_d - CoM;
        satellites{i}.velocity_d = satellites{i}.velocity_d - CoMV;
        disp("修正済み")
        disp(satellites{i}.position_d)
        disp(satellites{i}.velocity_d)

    end

