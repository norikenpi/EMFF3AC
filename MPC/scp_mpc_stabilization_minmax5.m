%% num基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定可能
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定
% シード設定
rng(1);

% 最終衛星間距離
d_target = 0.925;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%d_avoid = 0.18;
d_avoid = 0.18;
d_avoid = -1;

% 初期衛星間距離
d_initial = -0.18*1.1;

%sqrt_d_avoid = sqrt((0.18+0.01)^2/2);

% 制御可能範囲(m)
d_max = 1.0122;

% 衛星数　2基or5基or9基
num = 5;

% 衛星質量
m = 1; % 1
m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒
N = 100;

% trust region 
%delta = 0.1;

%ペアセット
pair_set = [1,2;
            2,3;
            3,4;
            4,5];


% 初期状態
s01 = [d_initial; d_initial; 0.0000001; 0; 0; 0];
s02 = [-d_initial; d_initial; -0.0000001; 0; 0; 0];
s03 = [-d_initial; -d_initial; 0.0000001; 0; 0; 0];
s04 = [d_initial; -d_initial; -0.0000001; 0; 0; 0];
s05 = [0.0000001; 0.0000001; 0.0000001; 0; 0; 0];
s0 = adjust_cog([s01, s02,s03, s04, s05], num); % 6num×1



%% Hill方程式 宇宙ステーション入門 P108

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% J2項
J2 = 1.08263e-3;
% 地球半径
Re = 6.371e6;
% 軌道半径
r_ref = 7.071e6;
% 軌道傾斜角
i_ref = 0;

var_s = 3*J2*Re^2*(1+3*cos(2*i_ref))/(8*r_ref^2);

c = sqrt(1+var_s);

% 1衛星に関する状態方程式の係数行列
% x_dot = A_ x + B_ u

A = [0, 0, 0, 1/2, 0, 0;
     0, 0, 0, 0, 1/2, 0;
     0, 0, 0, 0, 0, 1/2;
     (5*c^2-2)*n^2, 0, 0, 0, 2*n*c, 0;
     0, 0, 0, -2*n*c, 0, 0;
     0, 0, -(n*c)^2, 0, 0, 0]+...
    [(5*c^2-2)*n^2, 0, 0, 1, 2*n*c, 0;
     0, 0, 0, -2*n*c, 1, 0;
     0, 0, -(n*c)^2, 0, 0, 1;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0]/2; % 6×6

B = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m]*10^(-6); % 6×3

%2衛星に関する状態方程式の係数行
A_ = A;
B_ = B;
for i = 2:num
    A_ = blkdiag(A_, A); % BにAを対角に追加
    B_ = blkdiag(B_, B); % BにAを対角に追加
end

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

% 2衛星のそれぞれの目標状態

%目標レコード盤軌道の半径

rr1 = d_target/4;
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
disp("P生成")
tic
P = create_P(A_d, B_d, N, num); %6Nnum×3Nnum
%P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
elapsed_time = toc;
disp(['P生成 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);
disp("Q生成")
tic
Q = create_Q(A_d, N, num); %6N×6num
elapsed_time = toc;
disp(['Q生成 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);
%% 評価関数

%% 不等式制約

% 不等式制約1(全ての入力は最大入力以下)
% 最大入力との差が0より大きくなければならない。
%A1 = [eye(3*N*num), -ones(3*N*num,1); -eye(3*N*num), -ones(3*N*num, 1)]; %6N×3Nnum+1
%b1 = zeros(6*N*num, 1);%6Nnum×1

if not(d_avoid == -1)
    disp("A b生成")
    tic
    % 不等式制約2 (衛星間距離はd_avoid以上)
    % ノミナルの状態プロファイルを設定
    nominal_s = s;
    
    A2 = zeros(N*num*(num-1)/2, 3*num*N);
    b2 = zeros(N*num*(num-1)/2, 1);

    for i = 1:num-1
        for j = i+1:num
            % 状態ベクトルから衛星iと衛星jの位置ベクトルのみ抽出
            relative_mat_all = zeros(N, 6*num*N);
            relative_mat = zeros(3, 6*num);
            relative_mat(:,6*(i-1)+1:6*(i-1)+3) = eye(3);
            relative_mat(:,6*(j-1)+1:6*(j-1)+3) = -eye(3);
            for k = 1:N
                relative_mat_all(3*(k-1)+1:3*k, 6*num*(k-1)+1:6*num*k) = relative_mat;
            end
            % create_matrixは複数の相対位置ベクトルの内積をまとめて行うための行列を作っている。
            % 不等式の大小を変えるために両辺マイナスをかけている。
            A2(N*(i-1)+1:N*i,:) = -create_matrix(relative_mat_all * nominal_s).' * relative_mat_all * P; %500×3001
            b2(N*(i-1)+1:N*i) = -d_avoid * calculate_norms(relative_mat_all * nominal_s) + create_matrix(relative_mat_all * nominal_s).' * relative_mat_all * Q * s0;
        end
    end

    A = A2;
    b = b2;

    % 不等式制約3 (移動量はdelta以下)
    %A3 = [-P; P];
    %b3 = [delta * ones(6*N*num, 1) - s + Q * s0; delta * ones(6*N*num, 1) + s - Q * s0];
    
    %A = [A1; A2; A3];
    %b = [b1; b2; b3];
    elapsed_time = toc;
    disp(['A b生成 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);
    
end



%% 等式制約

% 等式制約1 (運動量保存)
disp("Aeq1 beq1生成")
tic
Aeq1 = create_Aeq1(N, num);
beq1 = zeros(3*N, 1);
elapsed_time = toc;
disp(['Aeq1 beq1 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);
% 等式制約2 (最終状態固定)
%Aeq2 = P(1:6*num,:);
%beq2 = sd - Q(1:6*num,:) * s0;


% 等式制約2 (相対軌道安定化) %5基なのでペア
disp("Aeq2 beq2生成")
tic
kA = 2e-3;
thetaP = pi/6;
relative_mat6 = [eye(6),-eye(6)];
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];



Aeq2 = zeros(4*4,3*N*5);
beq2 = zeros(4*4,1);

for j = 1:4 
    sat1 = pair_set(j,1);
    sat2 = pair_set(j,2);
    Aeq_ = mat * (P(6*(sat1-1)+1:6*sat1,:) - P(6*(sat2-1)+1:6*sat2,:));
    beq_ = zeros(4,1) - mat * (Q(6*(sat1-1)+1:6*sat1,:) - Q(6*(sat2-1)+1:6*sat2,:)) * s0;
    Aeq2(4*(j-1)+1:4*j,:) = Aeq_;
    beq2(4*(j-1)+1:4*j) = beq_;
end
elapsed_time = toc;
disp(['Aeq2 beq2生成 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

%% 線形不等式制約線形計画問題 
disp("最適化開始")
tic
% 解はnum×N×3自由度
cvx_begin sdp quiet
    variable x(3*N*num)
    minimize(max(x))
    
    pos = P * x + Q * s0; % 5*N*num

    subject to
        if not(d_avoid == -1)
            %進入禁止範囲設定
            A * x <= b;
        end
        
        %運動量保存＆最終状態で軌道安定完了
        Aeq * x == beq;

        for i = 1:N
            % ペア間最大距離制約　5基なので4ペア
            for j = 1:4
                sat1 = pair_set(j,1);
                sat2 = pair_set(j,2);
                sat1_pos = pos(6*5*(i-1)+6*(sat1-1)+1:6*5*(i-1)+6*(sat1-1)+3);
                sat2_pos = pos(6*5*(i-1)+6*(sat2-1)+1:6*5*(i-1)+6*(sat2-1)+3);
                rel_pos = sat1_pos - sat2_pos ;
                [d_max^2, rel_pos.';
                 rel_pos, eye(3)] >= 0;
            end
        end
cvx_end

elapsed_time = toc;
disp(['最適化 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);


% 衛星の状態
s = P * x + Q * s0;
s1 = s;
u = x;

disp("最大入力 u_max")
tic
u_max = max(abs(x))*10^(-6);
disp(u_max)
I_max_list = [];

disp("安定チェック")
tic
x_r = s(1:6)-s(7:12);
error = mat * x_r;
disp(error)
%% 図示
disp("図示開始")
tic
plot_s(s, num, N, rr, d_target, pair_set)
elapsed_time = toc;
disp(['図示 実行時間: ', num2str(floor(elapsed_time/60)), '分 ', num2str(rem(elapsed_time, 60)), '秒']);

%% 拘束条件チェック

disp("衛星間距離はd_avoid以上")
all_dist_list = calc_all_dist(num, N, s);
disp(min(all_dist_list))

disp("ペア組んでいる衛星間距離はd_max以下")
pair_dist_list = calc_pair_dist(N, pair_set, s);
disp(max(pair_dist_list))

disp("運動量保存")
disp(max(abs(beq - Aeq*x)))

disp("最終状態固定")
disp(max(abs(beq2 - Aeq2*x)))

%% 関数リスト

function all_dist_list = calc_all_dist(num, N, s)
    all_dist_list = zeros(N*num*(num-1)/2,1);
    data = reorderMatrix2(s);
    satellites = cell(1, num);

    % 衛星インスタンス生成
    for i = 1:num
        satellites{i} = zeros(N, 3);
    end
    
    % 衛星データ格納
    for i = 1:N
        for j = 1:num
            satellites{j}(i,:) = data(3*num*(i-1)+3*(j-1)+1:3*num*(i-1)+3*(j-1)+3).';
        end
    end
    for i = 1:num-1
        for j = i+1:num
            all_dist_list(N*((num-i/2)*(i-1)+(j-i)-1)+1:N*((num-i/2)*(i-1)+(j-i))) = vecnorm(satellites{i} - satellites{j}, 2, 2);
        end
    end

end

function pair_dist_list = calc_pair_dist(N, pair_set, s)
    pair_dist_list = zeros(4*N, 1);
    for i = 1:N
        % ペア間最大距離制約　5基なので4ペア
        for j = 1:4
            sat1 = pair_set(j,1);
            sat2 = pair_set(j,2);
            sat1_pos = s(6*5*(i-1)+6*(sat1-1)+1:6*5*(i-1)+6*(sat1-1)+3);
            sat2_pos = s(6*5*(i-1)+6*(sat2-1)+1:6*5*(i-1)+6*(sat2-1)+3);
            pair_dist_list(4*(i-1)+j) = norm(sat1_pos - sat2_pos);
        end
    end
end

function P = create_P(A, B, N, num)
    % 入力:
    % A: nxn の行列
    % B: nx1 のベクトル
    % N: 整数
    n = 6*num; % A行列の次元
    k = 3*num;
    P = zeros(n*N, k*N);
    for j = 1:N
        mat_i = zeros(n, k*N);
        %j個目までは0行列
        for i = 1:N
            if i >= 1 && i < j
                mat_i(:, k*(i-1)+1: k*i) = zeros(n, k);
            else
                mat_i(:, k*(i-1)+1: k*i) = A^(i-j) * B;
            end
        end

        P(n*(j-1)+1:n*j,:) = mat_i;
    end

end

function Q = create_Q(A, N, num)
    n = 6*num;
    Q = zeros(n*N, n);
    for j = 1:N
        mat_i = A^(N - j + 1);
        Q(n*(j-1)+1:n*j,:) = mat_i;
    end

end

function matrix_3n_n = create_matrix(vec_3n)
    % 複数の相対位置ベクトルの内積をまとめて行うための行列を作る
    % vec_3n: 3n x 1 ベクトル
    
    n = length(vec_3n) / 3; % 3次元ベクトルの個数
    matrix_3n_n = zeros(3*n, n); % 出力行列の初期化
    
    for i = 1:n
        % 各3次元ベクトルを抽出
        vec = vec_3n(3*(i-1)+1 : 3*i);
        % 対応するブロックに代入
        matrix_3n_n(3*(i-1)+1 : 3*i, i) = vec;
    end
end


function norms = calculate_norms(vec_3n)
    % vec_3n: 3n x 1 ベクトル
    
    n = length(vec_3n) / 3; % 3次元ベクトルの個数
    norms = zeros(n, 1); % 出力ベクトルの初期化
    
    for i = 1:n
        % 各3次元ベクトルを抽出
        vec = vec_3n(3*(i-1)+1 : 3*i);
        % ノルムを計算して保存
        norms(i) = norm(vec);
    end
end
%{
A2_ones = zeros(N*num, 6*N*num);
        for j = 1:N*num
            A2_ones(j, 6*(j-1)+1:6*(j-1)+3) = ones(1,3);
        end
%}
function Aeq1 = create_Aeq1(N, num)
    matrix1 = repmat(eye(3), 1, num);
    Aeq1 = zeros(3*N, 3*num*N);
    for i = 1:N
        Aeq1(3*(i-1)+1:3*i, 3*num*(i-1)+1:3*num*i) = matrix1;
    end
end

function s = adjust_cog(s_mat, num)
    
    cog = sum(s_mat, 2)/num;
    s = zeros(6*num, 1);
    for i = 1:num
        s(6*(i-1)+1: 6*i) = s_mat(:,i) - cog;
    end
end



function plot_s(s, num, N, rr, d_target, pair_set)
    % 2衛星の動画を表示。
    %3次元座標
    data = reorderMatrix2(s);
    satellites = cell(1, num);

    % 衛星インスタンス生成
    for i = 1:num
        satellites{i} = zeros(N, 3);
    end
    
    % 衛星データ格納
    for i = 1:N
        for j = 1:num
            satellites{j}(i,:) = data(3*num*(i-1)+3*(j-1)+1:3*num*(i-1)+3*(j-1)+3).';
        end
    end
    assignin('base', 'satellites', satellites)

    % ビデオライターオブジェクトの作成
    v = VideoWriter('points_motion_3D.avi'); % AVIファイル形式で動画を保存
    % 画質の設定（例：品質を最大に）
    v.Quality = 100;
    open(v);
    
    % フィギュアの作成
    figure;
    axis equal;
    xlim([-d_target*1.5, d_target*1.5]); % x軸の範囲を調整
    ylim([-d_target*1.5, d_target*1.5]); % y軸の範囲を調整
    zlim([-d_target*1.5, d_target*1.5]); % z軸の範囲を調整
    hold on;
    grid on; % グリッドを表示
    
    
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
            sat1 = pair_set(j,1);
            sat2 = pair_set(j,2);
            plot3([satellites{sat1}(i,1), satellites{sat2}(i,1)], [satellites{sat1}(i,2), satellites{sat2}(i,2)], [satellites{sat1}(i,3), satellites{sat2}(i,3)],  '-', 'Color', 'k', 'LineWidth', 2);
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


