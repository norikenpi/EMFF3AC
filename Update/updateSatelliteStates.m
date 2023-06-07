%(姿勢は考慮できてない)
% 各衛星に発生した磁気トルクと磁気力から、dt秒後の各衛星の状態量を求める
function satellites = updateSatelliteStates(satellites, param, magnetic_torques, magnetic_forces)
    %差分方程式をこっちでも実装しないといけない

    n = param.n;

    %オイラー近似の係数
    A = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 2*n;
         0, -n^2, 0, 0, 0, 0;
         0, 0, 3*n^2, -2*n, 0, 0];

    %台形近似の係数
    A = [0, 0, 0, 1/2, 0, 0;
         0, 0, 0, 0, 1/2, 0;
         0, 0, 0, 0, 0, 1/2;
         0, 0, 0, 0, 0, 2*n;
         0, -n^2, 0, 0, 0, 0;
         0, 0, 3*n^2, -2*n, 0, 0]+...
        [0, 0, 0, 0, 0, 2*n;
         0, -n^2, 0, 0, 0, 0;
         0, 0, 3*n^2, -2*n, 0, 0;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0]/2;
     
    %オイラー近似を利用した差分方程式の係数
    A_d = eye(6) + param.dt*A;

    for i = 1:param.N
        m = satellites{i}.mass;
        %Hill方程式によって作られるオイラー近似離散状態方程式の係数
        B = [0, 0, 0;
             0, 0, 0;
             0, 0, 0;
             1/m, 0, 0;
             0, 1/m, 0;
             0, 0, 1/m];
        
        %B_sharp = (B.'*B)\B.';
        %オイラー近似を利用した差分方程式の係数
        B_d = param.dt*B;
        position_before = satellites{i}.position;
        velocity_before = satellites{i}.velocity;


        state = A_d*[position_before; velocity_before] + B_d*magnetic_forces{i};
        % 位置の更新
        satellites{i}.position = state(1:3);
        % 速度の更新
        satellites{i}.velocity = state(4:6);
        % 姿勢の更新
        %satellites{i}.orientation = satellites{i}.orientation + satellites{i}.angular_velocity * dt;
        % 角速度の更新
        %satellites{i}.angular_velocity = satellites{i}.angular_velocity + (magnetic_torques{i} / satellites{i}.moment_of_inertia) * dt;
    end
end




