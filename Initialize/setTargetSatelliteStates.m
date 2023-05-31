
% 衛星の初期状態を設定する関数。位置、速度、オイラー角、角速度を初期化します。
function setTargetSatelliteStates(param)
    satellites = cell(1, param.N);
    for i = 1:param.N
        % ここで、各衛星の初期状態を設定してください
        
        %既定の範囲内で衛星の初期位置を設定
        %[position, velocity] = getSatellitePositionRandom(i, satellites, param);

        %衛星を2x2x(param.N/4)の配列に配置する
        [position_d, velocity_d] = getSatellitePosition(i, param);


        disp("position and velocity")
        disp(position_d);
        disp(velocity_d);
    end

    CoM = zeros(3,1);
    CoMV = zeros(3,1);

    %重心位置と重心速度を計算
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
    
end


