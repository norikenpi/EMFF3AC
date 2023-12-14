%% num基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定可能
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定

% 初期衛星間距離
d_initial = 0.4;
s01 = [-d_initial; -d_initial; 0.0000001; 0; 0; 0];
s02 = [d_initial; d_initial; -0.0000001; 0; 0; 0];


% 最終衛星間距離
d_target = 0.925;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%d_avoid = 0.18;
d_avoid = 0.8;
d_avoid = -1;

% 制御可能範囲(m)
d_max = 1.0122;
d_max = 1.0122*5;

% 衛星数　2基or5基or9基
num = 2;

% 衛星質量
m = 1; % 1
m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 10;

% trust region 
%delta = 0.1;



%% Hill方程式 宇宙ステーション入門 P108

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% 1衛星に関する状態方程式の係数行列
% x_dot = A_ x + B_ u
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

rr1 = d_target/4;
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];
s0 = adjust_cog([s01, s02], num); % 6num×1

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
P = create_P(A_d, B_d, N); %6Nnum×3Nnum
%P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
Q = create_Q(A_d, N); %6N×6num

%% 評価関数

%% 不等式制約

% 不等式制約1(全ての入力は最大入力以下)
% 最大入力との差が0より大きくなければならない。
%A1 = [eye(3*N*num), -ones(3*N*num,1); -eye(3*N*num), -ones(3*N*num, 1)]; %6N×3Nnum+1
%b1 = zeros(6*N*num, 1);%6Nnum×1

if not(d_avoid == -1)
    % 不等式制約2 (衛星間距離はd_avoid以上)
    % ノミナルの状態プロファイルを設定
    nominal_s = s;
    %{
    A2 = zeros(N*num*(num-1), 3*num*N);
    b2 = zeros(N*num*(num-1), 1);
    
    for i = 1:num-1
        for j = i:num
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
            b2(N*(i-1)+1:N*i,:) = -d_avoid * calculate_norms(relative_mat_all * nominal_s) + create_matrix(relative_mat_all * nominal_s).' * relative_mat_all * Q * s0;
        end
    end
    %}
    
    
    % 状態ベクトルから位置ベクトルのみを抽出
    C01 = [eye(3),zeros(3)];
    C1 = [];
    for i = 1:num*N
        C1 = blkdiag(C1, C01);
    end
    
    % 相対位置ベクトルを計算する行列
    C02 = [eye(3),-eye(3)];
    C2 = [];
    for i = 1:N
        C2 = blkdiag(C2, C02);
    end
    
    % create_matrixは複数の相対位置ベクトルの内積をまとめて行うための行列を作っている。
    % 不等式の大小を変えるために両辺マイナスをかけている。
    A2 = -create_matrix(C2 * C1 * nominal_s).' * C2 * C1 * P; %500×3001
    b2 = -d_avoid * calculate_norms(C2 * C1 * nominal_s) + create_matrix(C2 * C1 * nominal_s).' * C2 * C1 * Q * s0;
    
    A = A2;
    b = b2;

    % 不等式制約3 (移動量はdelta以下)
    %A3 = [-P; P];
    %b3 = [delta * ones(6*N*num, 1) - s + Q * s0; delta * ones(6*N*num, 1) + s - Q * s0];
    
    %A = [A1; A2; A3];
    %b = [b1; b2; b3];
end



%% 等式制約

% 等式制約1 (運動量保存)
Aeq1 = create_Aeq1(N, num);
beq1 = zeros(3*N, 1);

% 等式制約2 (最終状態固定)
%Aeq2 = P(1:6*num,:);
%beq2 = sd - Q(1:6*num,:) * s0;


% 等式制約2 (相対軌道安定化)
kA = 2e-3;
thetaP = pi/6;
relative_mat = [eye(6),-eye(6)];
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];
Aeq2 = mat * relative_mat * P(1:6*num,:);
beq2 = zeros(4,1) - mat * relative_mat * Q(1:6*num,:) * s0;



Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];
d_max_list = d_max * ones(N, 1);
%% 線形不等式制約線形計画問題 

% 解はnum×N×3自由度
cvx_begin sdp quiet
    variable x(3*N*num)
    minimize(max(x))
    
    pos = P * x + Q * s0;

    subject to
        if not(d_avoid == -1)
            A * x <= b;
        end

        Aeq * x == beq;
        for i = 1:N
            rel_pos = relative_mat(1:3,:) * pos(12*(i-1)+1:12*i);
            [d_max^2, rel_pos.';
             rel_pos, eye(3)] >= 0;
            
        end
cvx_end


% 衛星の状態
s = P * x + Q * s0;
s1 = s;
u = x;

disp("最大入力 u_max")
disp(max(abs(x))*10^(-6))
I_max_list = [];

disp("安定チェック")
x_r = s(1:6)-s(7:12);
error = mat * x_r;
disp(error)
%% 図示

plot_s(s, num, N, rr, d_target)

%% 関数リスト

% 関数の命名はテキトーです。すみません。

function P = create_P(A, B, N)
    % 入力:
    % A: nxn の行列
    % B: nx1 のベクトル
    % N: 整数
    n = size(A, 1); % A行列の次元
    k = size(B, 2);
    P = [];
    for j = 1:N
        mat_i = zeros(n, N);
        %j個目までは0行列
        for i = 1:N
            if i >= 1 && i < j
                mat_i(:, k*(i-1)+1: k*i) = zeros(n, k);
            else
                mat_i(:, k*(i-1)+1: k*i) = A^(i-j) * B;
            end
        end
        P = [P;mat_i];
    
    end

end

function Q = create_Q(A, N)
    Q = [];
    for j = 1:N
        mat_i = A^(N - j + 1);
        Q = [Q;mat_i];
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

function s0 = set_initialstates(num, d_initial)
    if num == 2
        s01 = [0.0000001; -d_initial; 0.0000001; 0; 0; 0];
        s02 = [-0.0000001; d_initial; -0.0000001; 0; 0; 0];
        s0 = adjust_cog([s01, s02], num); % 6num×1

    elseif num == 5
        s00 = [0.0000001; 0.0000001; 0.0000001; 0; 0; 0];
        s01 = [d_initial; 0.0000001; 0.0000001; 0; 0; 0];
        s02 = [-d_initial; -0.0000001; -0.0000001; 0; 0; 0];
        s03 = [-d_initial*2; -0.0000001; -0.0000001; 0; 0; 0];
        s04 = [d_initial*2; -0.0000001; -0.0000001; 0; 0; 0];
        s0 = adjust_cog([s00, s01, s02, s03, s04], num); % 6num×1
    elseif num == 9
        s00 = [0.0000001; 0.0000001; 0.0000001; 0; 0; 0];
        s01 = [d_initial; 0.0000001; 0.0000001; 0; 0; 0];
        s02 = [-d_initial; -0.0000001; -0.0000001; 0; 0; 0];
        s03 = [-d_initial*2; -0.0000001; -0.0000001; 0; 0; 0];
        s04 = [d_initial*2; -0.0000001; -0.0000001; 0; 0; 0];
        
        s05 = [3*d_initial; 0.0000001; 0.0000001; 0; 0; 0];
        s06 = [-3*d_initial; -0.0000001; -0.0000001; 0; 0; 0];
        s07 = [-4*d_initial; -0.0000001; -0.0000001; 0; 0; 0];
        s08 = [4*d_initial; -0.0000001; -0.0000001; 0; 0; 0];
    
        s0 = adjust_cog([s00, s01, s02, s03, s04, s05, s06, s07, s08], num); % 6num×1
    end
end

function sd = set_targetstates(num, rr, n, N, dt)
    if num == 2
        rr1 = rr(1);
        %sd1 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
        %sd3 = [-2*rr1*cos(n*N*dt + 2*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 2*2*pi/4); rr1*sin(n*N*dt + 2*2*pi/4); 2*n*rr1*sin(n*N*dt + 2*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 2*2*pi/4); n*rr1*cos(n*N*dt + 2*2*pi/4)];
        sd1 = [rr1*sin(n*N*dt); 2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); n*rr1*cos(n*N*dt); -2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt)];
        sd3 = [rr1*sin(n*N*dt + 2*2*pi/4); 2*rr1*cos(n*N*dt + 2*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 2*2*pi/4); n*rr1*cos(n*N*dt + 2*2*pi/4); -2*n*rr1*sin(n*N*dt + 2*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 2*2*pi/4)];
        sd = adjust_cog([sd1, sd3], num);
    elseif num == 5
        rr1 = rr(1);
        sd0 = [0.0000001; 0.0000001; 0.0000001; 0; 0; 0];
        sd1 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
        sd2 = [-2*rr1*cos(n*N*dt + 1*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 1*2*pi/4); rr1*sin(n*N*dt + 1*2*pi/4); 2*n*rr1*sin(n*N*dt + 1*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 1*2*pi/4); n*rr1*cos(n*N*dt + 1*2*pi/4)];
        sd3 = [-2*rr1*cos(n*N*dt + 2*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 2*2*pi/4); rr1*sin(n*N*dt + 2*2*pi/4); 2*n*rr1*sin(n*N*dt + 2*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 2*2*pi/4); n*rr1*cos(n*N*dt + 2*2*pi/4)];
        sd4 = [-2*rr1*cos(n*N*dt + 3*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 3*2*pi/4); rr1*sin(n*N*dt + 3*2*pi/4); 2*n*rr1*sin(n*N*dt + 3*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 3*2*pi/4); n*rr1*cos(n*N*dt + 3*2*pi/4)];
        
        sd = adjust_cog([sd0, sd1, sd2, sd3, sd4], num);
    elseif num == 9
        rr1 = rr(1);
        rr2 = rr(2);
    
        sd0 = [0.0000001; 0.0000001; 0.0000001; 0; 0; 0];
        sd1 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
        sd2 = [-2*rr1*cos(n*N*dt + 1*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 1*2*pi/4); rr1*sin(n*N*dt + 1*2*pi/4); 2*n*rr1*sin(n*N*dt + 1*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 1*2*pi/4); n*rr1*cos(n*N*dt + 1*2*pi/4)];
        sd3 = [-2*rr1*cos(n*N*dt + 2*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 2*2*pi/4); rr1*sin(n*N*dt + 2*2*pi/4); 2*n*rr1*sin(n*N*dt + 2*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 2*2*pi/4); n*rr1*cos(n*N*dt + 2*2*pi/4)];
        sd4 = [-2*rr1*cos(n*N*dt + 3*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 3*2*pi/4); rr1*sin(n*N*dt + 3*2*pi/4); 2*n*rr1*sin(n*N*dt + 3*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 3*2*pi/4); n*rr1*cos(n*N*dt + 3*2*pi/4)];
        
        
        sd5 = [-2*rr2*cos(n*N*dt + pi/4); sqrt(3)*rr2*sin(n*N*dt + pi/4); rr2*sin(n*N*dt + pi/4); 2*n*rr2*sin(n*N*dt + pi/4); sqrt(3)*n*rr2*cos(n*N*dt + pi/4); n*rr2*cos(n*N*dt + pi/4)];
        sd6 = [-2*rr2*cos(n*N*dt + 1*2*pi/4 + pi/4); sqrt(3)*rr2*sin(n*N*dt + 1*2*pi/4 + pi/4); rr2*sin(n*N*dt + 1*2*pi/4 + pi/4); 2*n*rr2*sin(n*N*dt + 1*2*pi/4 + pi/4); sqrt(3)*n*rr2*cos(n*N*dt + 1*2*pi/4 + pi/4); n*rr2*cos(n*N*dt + 1*2*pi/4 + pi/4)];
        sd7 = [-2*rr2*cos(n*N*dt + 2*2*pi/4 + pi/4); sqrt(3)*rr2*sin(n*N*dt + 2*2*pi/4 + pi/4); rr2*sin(n*N*dt + 2*2*pi/4 + pi/4); 2*n*rr2*sin(n*N*dt + 2*2*pi/4 + pi/4); sqrt(3)*n*rr2*cos(n*N*dt + 2*2*pi/4 + pi/4); n*rr2*cos(n*N*dt + 2*2*pi/4 + pi/4)];
        sd8 = [-2*rr2*cos(n*N*dt + 3*2*pi/4 + pi/4); sqrt(3)*rr2*sin(n*N*dt + 3*2*pi/4 + pi/4); rr2*sin(n*N*dt + 3*2*pi/4 + pi/4); 2*n*rr2*sin(n*N*dt + 3*2*pi/4 + pi/4); sqrt(3)*n*rr2*cos(n*N*dt + 3*2*pi/4 + pi/4); n*rr2*cos(n*N*dt + 3*2*pi/4 + pi/4)];
        
        
        sd = adjust_cog([sd0, sd1, sd2, sd3, sd4, sd5, sd6, sd7, sd8], num);
    end
end


function plot_s(s, num, N, rr, d_target)
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
function predicted_optimal_power = prediction(input,constants)
    Bias_nor_input=constants{1};
    Weight_nor_input=constants{2};
    weight1=constants{3};
    offset1=constants{4};
    weight2=constants{5};
    Weight_nor_label=constants{6};
    Bias_nor_label=constants{7};
    nor_input = (input- Bias_nor_input)*inv(Weight_nor_input);
    nor_label = double(weight2*max(weight1*nor_input.'+offset1, 0));
    predicted_optimal_power = Weight_nor_label*nor_label + Bias_nor_label;
end

function [power_per_EM_con,l] = cvx_optimization(d,Control_LOS,EM_con)
    cvx_begin sdp quiet
    variable l1 %nonnegative
    variable l2 %nonnegative
    variable l3 %nonnegative
    variable l4 %nonnegative
    variable l5 %nonnegative
    variable l6 %nonnegative
    power_per_EM_con = -[l1,l2,l3,l4,l5,l6]*Control_LOS/EM_con;
    minimize -power_per_EM_con
    subject to
    Rl = [-6*l1, 3*l2-d*l6,3*l3+d*l5;
        3*l2-2*d*l6,3*l1,-d*l4;
        3*l3+2*d*l5,d*l4,3*l1];
    [eye(3),Rl;Rl.',eye(3)]>=0;
    %-[l1,l2,l3,l4,l5,l6]*Control_LOS >=0
    cvx_end
    l = 1/EM_con*[l1;l2;l3;l4;l5;l6];
end

function [input,Control_LOS,d] = states_gene(d_max,d_min,f_max,f_min,tau_max,tau_min)
    if 1
        %
        d_sub = 1/sqrt(3)*(d_max-d_min)*(2*rand(3,1)-1);
        r10 = d_sub + d_min*d_sub/norm(d_sub);
        d = norm(r10);
        f_sub = 1/sqrt(3)*(f_max-f_min)*(2*rand(3,1)-1);
        force = f_sub + f_min*f_sub/norm(f_sub);
        if 1
            tau_sub = 1/sqrt(3)*(tau_max-tau_min)*(2*rand(3,1)-1);
            torque = tau_sub + tau_min*tau_sub/norm(tau_sub);
            Control = [force;torque];
            input = [r10;Control].';
            % new C_LOS
            C_los2o = C_LOS(r10,Control);
            r_los = C_los2o.'*r10;
            Control_LOS = [C_los2o.'*Control(1:3,1);C_los2o.'*Control(4:6,1)];
        else
            torque = (-1/2)*cross(r10,force);
            Control = [force;torque];
            % new C_LOS
            C_los2o = C_LOS(r10,Control);
            r_los = C_los2o.'*r10;
            Control = [force;torque];
            Control_LOS = [C_los2o.'*Control(1:3,1);C_los2o.'*Control(4:6,1)];
            Control_LOS(3)=0;Control_LOS(4)=0;Control_LOS(5)=0;
            input = [d,Control_LOS(1:2,:).',Control_LOS(4:6,:).'];
            input = input(1:3);
            %Inputs = [Inputs; [d,Control_LOS(1:2,:).',Control_LOS(4:6,:).']];
        end
        % elseif 1
        %     r10 = Position(num,:).';
        %     torque = (-1/2)*cross(r10,Force(num,:).');
        %     Control = [Force(num,:).';torque];
    end
end
function out = fun_Control_LOS(u,EM_con,Q)
    out=EM_con*Q*(kron([u(1);u(2);u(3)],[u(7);u(8);u(9)])+...
        kron([u(4);u(5);u(6)],[u(10);u(11);u(12)]));
end
function C_los2o = C_LOS(r10,Control)
    force = Control(1:3,1);
    e_x = r10/norm(r10);
    s = force-dot(e_x,force)*e_x;
    e_y = s/norm(s);
    e_z = cross(e_x,e_y);
    C_LOS_2_A = [e_x,e_y,e_z];
    C_los2o = C_LOS_2_A;
end
