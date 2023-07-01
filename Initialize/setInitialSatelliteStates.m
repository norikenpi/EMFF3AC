
% 衛星の初期状態を設定する関数。位置、速度、オイラー角、角速度を初期化します。
function satellites = setInitialSatelliteStates(param)
    satellites = cell(1, param.N);
    for i = 1:param.N
        % ここで、各衛星の初期状態を設定してください
        
        %既定の範囲内で衛星の初期位置を設定
        %[position, velocity] = getSatellitePositionRandom(i, satellites, param);

        %衛星を2x2x(param.N/4)の配列に配置する
        [position, velocity] = getSatellitePosition(i, param);

        [position, velocity] = getSatellitePositionCustom(i, param);

        %衛星の目標値を設定
        [position_d, velocity_d] = getSatelliteDesiredPosition(i, param);

        %velocity = [0.002*i; 0.001*i; 0.001*i]; % 速度
        orientation = [0; 0; i*pi/180]; % オイラー角
        angular_velocity = param.angular_velocity; % 角速度

        if param.current_type == "DC"
            magnetic_moment = param.magnetic_moment; % 磁気モーメント
        elseif param.current_type == "AC"
            magnetic_moment = zeros(3, size(param.frequency, 1)); % 磁気モーメント
        end
        mass = param.mass; % 衛星質量
        moment_of_inertia = param.moment_of_inertia; % 慣性モーメント
        max_magnetic_moment = param.max_magnetic_moment; % 最大磁気モーメント
        radius = param.radius; %衛星半径
        frequency = param.frequency_set(1,:);

        satellites{i} = Satellite(position, velocity, orientation, angular_velocity, magnetic_moment, ...
            mass, moment_of_inertia, max_magnetic_moment, radius, position_d, velocity_d, frequency);

    end


    satellites = adjustSatelliteState(satellites, param);
    satellites = adjustSatelliteDesiredState(satellites, param);
    
end


