
% 衛星の初期状態を設定する関数。位置、速度、オイラー角、角速度を初期化します。
function satellites = setInitialSatelliteStates(param)
    satellites = cell(1, param.N);
    for i = 1:param.N
        % ここで、各衛星の初期状態を設定してください
        
        %position = getSatellitePosition(i, param); % 位置
        %velocity = [0; 0; 0]; % 速度
        
        [position, velocity] = getSatellitePositionRandom(i, satellites, param);
        disp("position and velocity")
        disp(position);
        disp(velocity);
        %velocity = [0.002*i; 0.001*i; 0.001*i]; % 速度
        orientation = [0; 0; i*pi/180]; % オイラー角
        angular_velocity = [0; 0; 0.1*i]; % 角速度
        magnetic_moment = [0; 0.01; 0]; % 磁気モーメント
        mass = 0.01; % 衛星質量
        moment_of_inertia = 1; % 慣性モーメント
        max_magnetic_moment = 0.01; % 最大磁気モーメント
        radius = 0.05;

        satellites{i} = Satellite(position, velocity, orientation, angular_velocity, magnetic_moment, mass, moment_of_inertia, max_magnetic_moment, radius);
    end

    CoM = zeros(3,1);
    CoMV = zeros(3,1);
    for i = 1:param.N
        CoM = CoM + satellites{i}.position/param.N;
        CoMV = CoMV + satellites{i}.velocity/param.N;
        disp('CoM')
        disp(CoM)
        disp('CoMV')
        disp(CoMV)
    end
    
    for i = 1:param.N
        satellites{i}.position = satellites{i}.position - CoM;
        satellites{i}.velocity = satellites{i}.velocity - CoMV;
        disp("修正済み")
        disp(satellites{i}.position)
        disp(satellites{i}.velocity)

    end
    
end


