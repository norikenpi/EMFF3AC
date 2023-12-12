%% num基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定可能
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定

% 初期衛星間距離
d_initial = 0.3;

% 最終衛星間距離
d_target = 0.925;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%d_avoid = 0.01;
d_avoid = 0.299;
d_avoid = -1;

% 衛星数　2基or5基or9基
num = 2;

% 衛星質量
m = 1; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 100;

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
     0, 0, 0, 0, 0, 2*n;
     0, -n^2, 0, 0, 0, 0;
     0, 0, 3*n^2, -2*n, 0, 0]+...
    [0, 0, 0, 1, 0, 2*n;
     0, -n^2, 0, 0, 1, 0;
     0, 0, 3*n^2, -2*n, 0, 1;
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



% 初期状態
if num == 2
    d_initial = d_initial/2;
end
s0 = set_initialstates(num, d_initial);
s01 = [-d_initial; -d_initial; 0.0000001; 0; 0; 0];
s02 = [d_initial; d_initial; -0.0000001; 0; 0; 0];
s0 = adjust_cog([s01, s02], num); % 6num×1

% 2衛星のそれぞれの目標状態
%目標レコード盤軌道の半径
rr1 = d_target/2;
if num == 2
    rr1 = d_target/4;
end
rr2 = sqrt(2)*d_target/2;

rr = [rr1,rr2];

%rr = create_rr(num)

sd = set_targetstates(num, rr, n, N, dt);

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
P = create_P(A_d, B_d, N); %6Nnum×3Nnum
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
Q = create_Q(A_d, N); %6N×6num

%% 評価関数

% 評価関数1(最大入力最小)
f1 = [zeros(1, 3*N*num), 1]; 
f = f1;

%% 不等式制約

% 不等式制約1(全ての入力は最大入力以下)
% 最大入力との差が0より大きくなければならない。
A1 = [eye(3*N*num), -ones(3*N*num,1); -eye(3*N*num), -ones(3*N*num, 1)]; %6N×3Nnum+1
b1 = zeros(6*N*num, 1);%6Nnum×1

A = A1;
b = b1;

if not(d_avoid == -1)
    % 不等式制約2 (衛星間距離はR以下)
    % ノミナルの状態プロファイルを設定
    nominal_s = s;
    
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
    
    A = [A1; A2];
    b = [b1; b2];

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
Aeq2 = P(1:6*num,:);
beq2 = sd - Q(1:6*num,:) * s0;
Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

%% 線形不等式制約線形計画問題 
% 解はnum×N×3自由度

% linprogを使う場合

[x,fval,exitflag,output,lambda] = ...
   linprog(f, A, b, Aeq, beq);


% cvxを使う場合
%{
cvx_begin sdp quiet
    variable x(size(f, 2))
    minimize(f * x)
    subject to
        A * x <= b;
        Aeq * x == beq;
cvx_end
%}

% 衛星の状態
s = P * x + Q * s0;
s1 = s;
u = x;

disp("最大入力 u_max")
disp(x(3*num*N+1)*10^(-6))
I_max_list = [];

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
    Aeq1 = zeros(3*N, 3*num*N+1);
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
        s01 = [d_initial; 0.0000001; 0.0000001; 0; 0; 0];
        s02 = [-d_initial; -0.0000001; -0.0000001; 0; 0; 0];
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
        sd1 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
        sd3 = [-2*rr1*cos(n*N*dt + 2*2*pi/4); sqrt(3)*rr1*sin(n*N*dt + 2*2*pi/4); rr1*sin(n*N*dt + 2*2*pi/4); 2*n*rr1*sin(n*N*dt + 2*2*pi/4); sqrt(3)*n*rr1*cos(n*N*dt + 2*2*pi/4); n*rr1*cos(n*N*dt + 2*2*pi/4)];
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
