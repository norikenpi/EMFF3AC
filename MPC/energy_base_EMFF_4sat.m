
clear
%% パラメータ設定
% 最終衛星間距離
d_target = 0.925;

% 衛星数　2基or5基or9基
num = 4;

% 衛星質量
m = 1; % 1
%m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 10;
N = 1;
n = 0.0011; % 0.0011
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
d_initial = d_avoid/2;

rr1 = d_target/(2*sqrt(3));
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];


disp("最大電力設定")
disp(P_max)
disp("最大電流設定")
disp(I_max)
disp("最大磁気モーメント設定")
disp(myu_max)

param.d_target = d_target;
param.num = num;
param.m = m; 
param.dt = dt;
param.N = N;
param.n = n;
param.coilN = coilN;
param.radius = radius;
param.P_max = P_max; % W
param.rho = rho; % Ω/m
param.wire_length = wire_length;
param.wire_S = wire_S;
param.R_rho = R_rho; 
param.I_max = I_max;
param.myu_max = myu_max;
param.d_avoid = d_avoid;
param.d_initial = d_initial;
param.rr = rr;

satellites = cell(1, num);
pair_set = zeros(4, 2, N);  % 4x2xTのゼロ配列の初期化


s01 = [-d_initial; -d_initial; 0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s02 = [-d_initial; d_initial; -0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s03= [d_initial; d_initial; 0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s04 = [d_initial; -d_initial; -0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s0 = adjust_cog([s01, s02, s03, s04], num); % 6num×1

%各衛星初期状態決定
s01 = s0(1:6);
s02 = s0(7:12);
s03 = s0(13:18);
s04 = s0(19:24);

E_border = 5;
E_all = 100;

state1 = zeros(N,6);
state2 = zeros(N,6);
state3 = zeros(N,6);
state4 = zeros(N,6);

state1(1,:) = s01.';
state2(1,:) = s02.';
state3(1,:) = s03.';
state4(1,:) = s04.';
state_mat  = [s01.';s02.';s03.';s04.'];
state_mat0 = state_mat;
%% シミュレーション
%エネルギーの総和がE_border以下だったら問題ない。
i = 0;



while E_all > E_border
    i = i + 1;
    N = i; 
    % ペア組み
    
    pair_mat = make_pair(state_mat, param);
    pair_set(:,:,i) = pair_mat;

    % 最大電力割り当て
    % ペアごとに割り振る感じがいいのかな。
    counts = count_pair_num(pair_mat);

    % 各ペア制御入力計算（for ペアlist　座標平行移動）
    % ペアでforを回して、一発で現在の状態と最大磁気モーメントから制御入力とペア間に働く力が出る関数がほしい。
    % 最適化ベースになるとここが変わるだけ。
    pair_mat_thrust = calc_pair_thrust(pair_mat, counts, state_mat, param);
    %disp(pair_mat_thrust)

    % 推力計算
    % ペアでforを回して、各衛星に働く力を計算。
    thrust_mat = calc_thrust(pair_mat, pair_mat_thrust);
    %thrust_mat = calc_optimal_thrust(pair_mat, pair_mat_thrust);
    
    % 状態量更新
    % 現在の状態量と推力を入れたら次の時刻の状態量が出てくる関数
    state_mat = update_state(state_mat, thrust_mat, param);
    
    state1(i+1,:) = state_mat(1,:);
    state2(i+1,:) = state_mat(2,:);
    state3(i+1,:) = state_mat(3,:);
    state4(i+1,:) = state_mat(4,:);

    %総エネルギー計算
    %次の時刻の状態から総エネルギーを計算

    E_all = calc_E_all(state_mat, param);
    disp("エネルギー総和")
    disp(N)
    disp(E_all)
    if N == 5000
        %ペアを組んでいる衛星が
        break
    end
end
disp("シミュレーション終了")
disp("シミュレーション時間")
total_seconds = N*dt;

% 時間、分、秒への変換
hours = floor(total_seconds / 3600);
minutes = floor(mod(total_seconds, 3600) / 60);
seconds = mod(total_seconds, 60);

% 結果の表示
disp(['時間: ' num2str(hours) ' 時間 ' num2str(minutes) ' 分 ' num2str(seconds) ' 秒']);

%% 図示
satellites{1} = state1(:,1:3);
satellites{2} = state2(:,1:3);
satellites{3} = state3(:,1:3);
satellites{4} = state4(:,1:3);

plot_s(satellites, num, N, rr, d_target, pair_set)

%% 関数リスト

function pair_mat = make_pair(state_mat, param)
    pair_mat = [1,1;
                2,2;
                3,3;
                4,4];

    pair_candidate = calc_pair_candidate(state_mat, param);
    for i = 1:4
        delta_E_max = 0;
        pair = 0;
        for j = pair_candidate{i}
            state = state_mat(j,:) - state_mat(i,:);
            delta_E = calc_E(state, param);
            %disp(delta_v)
            if delta_E > delta_E_max
                delta_E_max = delta_E;
                pair = j;
            end
            
        end
        pair_mat(i,2) = pair;
    end
    
%{
    pair_mat = [1,2;
                2,3;
                3,4;
                4,1];
%}

end

function pair_candidate = calc_pair_candidate(state_mat, param)
    radius = param.radius;
    X = state_mat(:,1:3);
    % KDツリーを構築
    tree = KDTreeSearcher(X);
    pair_candidate = cell(1, 4);
    for i = 1:4
        queryIndex = i;
        queryPoint = X(queryIndex, :);
        % 検索範囲を設定（例：0.2）
        range = 1;
    
        % 最小距離を設定（例：0.05）
        minDistance = radius*6;
        % 検索範囲内の点のインデックスを取得
        idx = rangesearch(tree, queryPoint, range);
        
        % 最小距離よりも遠い点をフィルタリング
        pair_candidate{i} = idx{1}(vecnorm(X(idx{1}, :) - queryPoint, 2, 2) > minDistance);
        
    end
end


function counts = count_pair_num(pair_mat)
    counts = histcounts(pair_mat, 1:5); 
end

function pair_mat_thrust = calc_pair_thrust(pair_mat, counts, state_mat, param)
    pair_mat_thrust = zeros(3, 2, 4);
    for i = 1:4
        sat1 = pair_mat(i,1);
        sat2 = pair_mat(i,2);
        X = state_mat(sat1,:).' - state_mat(sat2,:).';
        u = calc_u(X, param);
        pair_mat_thrust(:,1,i) = u/(counts(sat1)*counts(sat2));
        pair_mat_thrust(:,2,i) = -u/(counts(sat1)*counts(sat2));
    end
end

function thrust_mat = calc_thrust(pair_mat, pair_mat_thrust)
    thrust_mat = zeros(3,4);
    for i = 1:4 
        [row, col] = find(pair_mat == i);
        thrust_sum = zeros(3,1);
        for j = 1:length(row)
            thrust_sum = thrust_sum + pair_mat_thrust(:,col(j),row(j));
        end
        thrust_mat(:,i) = thrust_sum;
    end
end

function state_mat = update_state(state_mat, thrust_mat, param)
    n = param.n;
    m = param.m;
    dt = param.dt;
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
    for i = 1:4
        u = thrust_mat(:,i);
        state_mat(i,:) = (A_d *  state_mat(i,:).' + B_d * u).';
    end
end

function E_all = calc_E_all(state_mat, param)
    E_all = 0;
    for i = 1:4
        state = state_mat(i,:);
        E_all = E_all + calc_E(state, param); 
    end
end

function E = calc_E(state, param)
    n = param.n;
    kA = 2e-3;
    thetaP = pi/6;
    %relative_mat = [eye(6),-eye(6)];
    
    mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
           2,0,0,0,1/n,0;
           0,0,0,-1/(n*tan(thetaP)),0,1/n;
           -1/(n*tan(thetaP)),0,1/n, 0,0,0];
    E = sum(abs(mat * state.'));
end

function u = calc_u(X, param)
    d_avoid = param.d_avoid;
    n = param.n;
    coilN = param.coilN;
    radius = param.radius;
    I_max = param.I_max;
    myu_max = param.myu_max;
    X = X/2;
    if norm(X(1:3)) > d_avoid/2
        C110 = coord2const(X, n);
        kA = 2;%2e-3;
        kB = 1;%1e-3;
        C1 = C110(1); C4 = C110(4); C5 = C110(5);
        C2 = C110(2); C3 = C110(3);
        r_xy = C110(7); phi_xy = C110(8); %phi_xy = atan2(o_r_ji(1),o_r_ji(2)/2);
        C4d = 3*n*C1/kA; %目標値
        dC4 = C4-C4d; %C4偏差
        thetaP = pi/6;
        C5d = C2/tan(thetaP);
        u_A = n*[1/2*dC4;-C1];  
        u = [kA*u_A;-kB*n*(C5-C5d)];
        r = X(1:3).'*2; % 2衛星を考慮して2倍にする
        [myu1, myu2] = ru2myu(r,u, coilN, radius, I_max);
        %u = [0;0;0];
        if norm(myu1) > myu_max
            u = myu_max* u/norm(myu1);
            myu1 = myu_max/2 * myu1/norm(myu1);
            %disp("over myu1")
        end
        
    else 
        k_avoid = 1e-1;
        u = k_avoid * 10000 * X(1:3)/norm(X(1:3));
        r = X(1:3).'*2;
        [myu1, myu2] = ru2myu(r,u, coilN, radius, I_max);
        if norm(myu1) > myu_max
            u = myu_max* u/norm(myu1);
            myu1 = myu_max/2 * myu1/norm(myu1);
            %disp("over myu1")
        end
        disp("avoid")
        disp(norm(X(1:3)))
    end
end

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


function s = adjust_cog(s_mat, num)
    
    cog = sum(s_mat, 2)/num;
    s = zeros(6*num, 1);
    for i = 1:num
        s(6*(i-1)+1: 6*i) = s_mat(:,i) - cog;
    end
end

function plot_s(satellites, num, N, rr, d_target, pair_set)
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
    for i = 1:10:N
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

        for j = 1:4
            % ペアリングを表示
            sat1 = pair_set(j,1,i);
            sat2 = pair_set(j,2,i);
            plot3([satellites{sat1}(i,1), satellites{sat2}(i,1)], [satellites{sat1}(i,2), satellites{sat2}(i,2)], [satellites{sat1}(i,3), satellites{sat2}(i,3)],  '-', 'Color', 'k', 'LineWidth', 2);
        end

        % 視点を変更
        azimuth = 135; % 方位角
        elevation = 30; % 仰角
        view(azimuth, elevation);
    
        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    
    % ビデオの保存
    close(v);
    
end