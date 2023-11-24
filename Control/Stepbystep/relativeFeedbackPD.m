function [u, satellites, histories] = relativeFeedbackPD(i, pair_satellite_idx, satellites, param, histories)
    %現在の相対位置速度と目標の相対位置速度
    relative_position = satellites{pair_satellite_idx}.position - satellites{i}.position;
    relative_velocity = satellites{pair_satellite_idx}.velocity - satellites{i}.velocity;

    relative_position_d = satellites{pair_satellite_idx}.position_d - satellites{i}.position_d;
    relative_velocity_d = satellites{pair_satellite_idx}.velocity_d - satellites{i}.velocity_d;
    %relative_accelaration_d = [0;0;0];
    
    %目標値と現在地のずれ
    e = relative_position_d - relative_position;
    e_dot = relative_velocity_d - relative_velocity;

    %一定の領域内の誤差を仮定
    delta = 0.01;
    delta_dot = 0.001;
    
    %定数
    myu0 = param.myu0;
    d = param.satellite_desired_distance;
    N = param.coilN;
    R = param.radius;
    m = param.mass;
    Imax = param.I_max;
    
    %ゲイン
    k = 1/4;

    C = 3*myu0*N*R^2/(4*m);

    %k0_pos = k * C * Imax^2/(d^4 * delta);
    %k0_vel = k * C * Imax^2/(d^4 * delta_dot);

    k0_pos = param.Kp * C * Imax^2/(d^4);
    k0_vel = param.Kd * C * Imax^2/(d^4);

    u = - k0_pos * e - k0_vel * e_dot ; 
    %disp(u)

end
