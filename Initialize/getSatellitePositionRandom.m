% 衛星をランダムに配置する
%C1~C6 rand([-0.1; 0.1])
function [pos, vel] = getSatellitePositionRandom(i, satellites, param)
    j = param.j;
    rng(i+j);
    C1 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    rng(i+j+1);
    C2 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    rng(i+j+2);
    C3 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    rng(i+j+3);
    C4 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    rng(i+j+4);
    C5 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    rng(i+j+5);
    C6 = -param.C1_ini + (param.C1_ini * 2).*rand(1,1);
    if i == 1
        relative_position = zeros(3,1);
        relative_velocity = zeros(3,1);
    else
        relative_position = satellites{i-1}.position;
        relative_velocity = satellites{i-1}.velocity;
    end
    

    x = C4 - 2*C2 + relative_position(1);
    y = C6 + relative_position(2);
    z = -(2*C1 + C3) + relative_position(3);
    vx = -param.n*(3*C1 + 2*C3) + relative_velocity(1);
    vy = param.n*C5 + relative_velocity(2);
    vz = param.n*C2 + relative_velocity(3);
    pos = [x; y; z];
    vel = [vx; vy; vz];
end
