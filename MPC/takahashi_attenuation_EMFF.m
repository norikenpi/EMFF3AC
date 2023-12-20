%satteliteというcell 配列を入れてそこに位置を記録していこう。
%高橋座標系になっていることに注意。

num = 1;
dt = 10;
N = 250;
n = 0.0011; % 0.0011
m = 1; % 1
%u_max = 1e-9;
coilN = 140;
radius = 0.05;
P_max = 10; % W
rho = 1.68e-7; % Ω/m
wire_length = 140*0.05*2*pi;
wire_S = (0.2e-3)^2*pi;
R_rho = rho * wire_length/wire_S; 
I_max = sqrt(P_max/R_rho);

myu_max = I_max * coilN * radius^2 * pi;

d_avoid = radius*6;
% 初期衛星間距離
d_initial = d_avoid/2;

s0 = [d_initial; d_initial; -0.00005; 0; 0; 0];

u_list = zeros(3*N,1);
myu_list = zeros(3*N,1);
myu_list2 = zeros(3*N,1);

A = [0, 0, 0, 1/2, 0, 0;
     0, 0, 0, 0, 1/2, 0;
     0, 0, 0, 0, 0, 1/2;
     3*n^2, 0, 0, 0, 2*n, 0;
     0, 0, 0, -2*n, 0, 0;
     0, 0, -n^2, 0, 0, 0]+...
    [3*n^2, 0, 0, 1, 2*n, 0;
     0, 0, 0, -2*n, 1, 0;
     0, 0, -n^2, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0]/2; 

B = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m]; % 6×3

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6)+ dt*A; % 6num×6num
B_d = dt*B; % 6num×3num



d_target = 0.925;

rr1 = d_target/(2*sqrt(3));
if num == 2
    rr1 = d_target/4;
end
rr2 = sqrt(2)*d_target/2;

rr = [rr1,rr2];
rr1 = rr(1);
%s0 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
%s0 = [rr1*sin(0); 2*rr1*cos(0); sqrt(3)*rr1*sin(0); n*rr1*cos(0); -2*n*rr1*sin(0); sqrt(3)*n*rr1*cos(0)];

state = zeros(N,6);
state(1,:) = s0.';
s = zeros(6*N*2,1);
thetaP = pi/6;
rd = 0;

for i = 1:N
    X = state(i,:).';
    if norm(X(1:3)) > d_avoid/2
        C110 = coord2const(X, n);
        kA = 2;%2e-3;
        kB = 1;%1e-3;
        C1 = C110(1); C4 = C110(4); C5 = C110(5);
        C2 = C110(2); C3 = C110(3);
        r_xy = C110(7); phi_xy = C110(8); %phi_xy = atan2(o_r_ji(1),o_r_ji(2)/2);
        C4d = 3*n*C1/kA; %目標値
        dC4 = C4-C4d; %C4偏差
        C5d = C2/tan(thetaP);
        u_A = n*[1/2*dC4;-C1];  
        u = [kA*u_A;-kB*n*(C5-C5d)];
        r = state(i,1:3).'*2; % 2衛星を考慮して2倍にする
        [myu1, myu2] = ru2myu(r,u, coilN, radius, I_max);
        %u = [0;0;0];
        if norm(myu1) > myu_max
            u = myu_max/2* u/norm(myu1);
            myu1 = myu_max/2 * myu1/norm(myu1);
            disp("over myu1")
        end
        
    else 
        k_avoid = 1e-1;
        u = k_avoid * u_max * X(1:3)/norm(X(1:3));
        disp("avoid")
    end
    myu_list(3*N-3*(i-1)-2:3*N-3*(i-1)) = myu1;
    state(i+1,:) = (A_d * state(i,:).' + B_d * u).';
    s(6*N*2 - 6*2*(i-1)-11:6*N*2 - 6*2*(i-1)-6) = (A_d * state(i,:).' + B_d * u);
    s(6*N*2 - 6*2*(i-1)-5:6*N*2 - 6*2*(i-1)) = -(A_d * state(i,:).' + B_d * u);

end

satellites{1} = state(:,1:3);
plot_s(satellites, num, N, rr, d_target)
disp("安定チェック")
x_r = state(N+1,:).';
kA = 2e-3; % 2e-3
thetaP = pi/6;
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];
error = mat * x_r;
disp(error)
disp("最小化したい評価値")
disp(sum(abs(error)))


function C = coord2const(X, w)
    %% HCW constants calculated from the free motion equation
    C(1) = 2*X(1)+X(5)/w;
    C(2) = X(4)/w;
    C(3) = -3*X(1)-2*X(5)/w;
    C(4) = X(2)-2*X(4)/w;
    C(5) = X(6)/w;
    C(6) = X(3);
    %%
    r_xy = sqrt(C(2)^2+C(3)^2);
    theta_xy = atan2(C(3),C(2));
    r_z = sqrt(C(5)^2+C(6)^2);
    theta_z = atan2(C(6),C(5));
    C(7) = r_xy;
    C(8) = theta_xy;
    C(9) = r_z;
    C(10) = theta_z;
end

function plot_s(satellites, num, N, rr, d_target)
    % 2衛星の動画を表示。
    %3次元座標

    % ビデオライターオブジェクトの作成
    v = VideoWriter('points_motion_3D.avi'); % AVIファイル形式で動画を保存
    % 画質の設定（例：品質を最大に）
    v.Quality = 100;
    open(v);
    
    % フィギュアの作成
    figure;
    axis equal;
    xlim([-d_target*1.5, d_target*1.5]/3); % x軸の範囲を調整
    ylim([-d_target*1.5, d_target*1.5]/3); % y軸の範囲を調整
    zlim([-d_target*1.5, d_target*1.5]/3); % z軸の範囲を調整
    hold on;
    grid on; % グリッドを表示
    
    set(gca, 'ZDir', 'reverse')
    
    % 軸のラベルを設定
    xlabel('X[m](地心方向)');
    ylabel('Y[m](軌道進行方向)');
    zlabel('Z[m](軌道面垂直方向)');
    
    theta = linspace(0, 2 * pi, 100); % 0から2πまでの角度を生
    colors = hsv(num); % HSVカラースペースを使用してN個の異なる色を生成
    
    % 各フレームでの点の位置をプロットし、そのフレームを動画に書き込む
    for i = 1:1:N
        cla;
        
        for j = 1:length(rr)
            % レコード盤軌道をプロット
            %x1 = -2*rr(j)*cos(theta); % x座標を計算
            %y1 = sqrt(3)*rr(j)*sin(theta); % y座標を計算
            %z1 = rr(j)*sin(theta);
            x1 = rr(j)*sin(theta);% x座標を計算
            y1 = 2*rr(j)*cos(theta); % y座標を計算2*rr(j)*cos(theta);
            z1 = sqrt(3)*rr(j)*sin(theta);
            plot3(x1, y1, z1, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1); % 円を灰色で描画
        end

        % x軸方向の点線を描画
        y_position = -1:0.1:1; % x軸方向の点線のx座標を指定
        x_position = zeros(size(y_position)); % y座標はすべて0に設定
        plot(x_position, y_position, 'k--', 'LineWidth', 1.5); % 点線を描画
       
        for j = 1:num
            % 軌道をプロット
            plot3(satellites{j}(1:N,1), satellites{j}(1:N,2), satellites{j}(1:N,3), '-', 'Color', colors(j,:));
            % 衛星をプロット
            plot3(satellites{j}(i,1), satellites{j}(i,2), satellites{j}(i,3), '.', 'MarkerSize', 60, 'Color', colors(j,:));
            % 衛星の初期値をプロット
            plot3(satellites{j}(1,1), satellites{j}(1,2), satellites{j}(1,3), 'o', 'MarkerSize', 5, 'Color', colors(j,:));
        end
        % 視点を変更
        azimuth = 225; % 方位角
        elevation = 30; % 仰角
        view(azimuth, elevation);
    
        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    
    % ビデオの保存
    close(v);
    
end


function B = reorderMatrix2(A)
    idx = find(mod(1:length(A), 6) == 1 | mod(1:length(A), 6) == 2 | mod(1:length(A), 6) == 3);
    A = flip(A(idx));
    B = [];
    %A = 1:100;
    n = 3;
    for i = 1:n:length(A)
        sub_vector = A(i:min(i+n-1, length(A)));
        B = [B; flip(sub_vector)];
    end
end


function [u_myu1, u_myu2] = force2moment(s, s0, num, u, coilN, radius, I_max)
    % 最終状態時以外の状態を抽出
    s = [s(1+6*num:end);s0];
    % 位置情報のみを抽出
    pos_vec = extract_3elements(s);
    % 相対位置を計算
    r = calc_rel_pos(pos_vec);
    u_myu1 = [];
    u_myu2 = [];
    i = 1;
    n = length(r);
    while i <= n
        [myu1, myu2] = ru2myu(r(i:i+2), u(i:i+2), coilN, radius, I_max);
        %片方だけ保存しておけばもう片方は、相対位置ベクトルと最大入力から再現できる。
        u_myu1 = [u_myu1; myu1];
        u_myu2 = [u_myu2; myu2];
        i = i + 6;
    end
end

function [myu1, myu2] = ru2myu(r,u, coilN, radius, I_max)
    % 原点の2倍の距離で計算
    r_norm = norm(r); 
    myu01 = coilN * pi * radius^2 * I_max * r/r_norm;
    myu0 = 4*pi*1e-7; % 真空の透磁率
    
    D = calculateD(r, myu01);
    %myu02 = 4*pi*r_norm^5/(3*myu0)*inv(D)*u*mass;
    myu02 = 4*pi*r_norm^5/(3*myu0)*inv(D)*u;
    myu1 = myu02;
    myu2 = myu01;
end


function D = calculateD(r, m1)
    x = r(1);
    y = r(2);
    z = r(3);
    r_norm = norm(r);
    D11 = 2*m1(1)*x + dot(m1, r) - 5*dot(m1, r)*x^2/r_norm^2;
    D12 = m1(2)*x + m1(1)*y - 5*dot(m1, r)*y*x/r_norm^2;
    D13 = m1(3)*x + m1(1)*z - 5*dot(m1, r)*z*x/r_norm^2;
    D21 = D12;
    D22 = 2*m1(2)*y + dot(m1, r) - 5*dot(m1, r)*y^2/r_norm^2;
    D23 = m1(3)*y + m1(2)*z - 5*dot(m1, r)*z*y/r_norm^2;
    D31 = D13;
    D32 = D23;
    D33 = 2*m1(3)*z + dot(m1, r) - 5*dot(m1, r)*z^2/r_norm^2;


    D = [D11,D21,D31;
         D12,D22,D32;
         D13,D23,D33;];

end


function rel_pos = calc_rel_pos(pos_vec)
    n = length(pos_vec); % ベクトルの長さを取得
    rel_pos = []; % 抽出された要素を格納するための空のベクトル
    i = 1;
    while i <= n
        pos_vec1 = pos_vec(i:i+2);
        pos_vec2 = pos_vec(i+3:i+5);
        rel_pios1 = pos_vec1 - pos_vec2;
        rel_pos2 = -rel_pios1;
        rel_pos = [rel_pos; rel_pios1; rel_pos2];
        i = i + 6;
    end
end


function extracted = extract_3elements(vector)
    n = length(vector); % ベクトルの長さを取得
    extracted = []; % 抽出された要素を格納するための空のベクトル

    i = 1;
    while i <= n
        % 現在の位置から3要素を抽出（範囲外にならないようにチェック）
        end_index = min(i+2, n);
        extracted = [extracted; vector(i:end_index)];
        
        % 6要素分進める（3要素抽出して3要素スキップ）
        i = i + 6;
    end
end
