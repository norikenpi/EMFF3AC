%% 3基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 衝突防止のための侵入禁止範囲は設定できていない。全ての衛星ての組み合わせを考えるのが面倒なため。
 
% 最大入力を最小化
% 進入禁止範囲を設定
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% 計算にだいぶ時間がかかるので、もっとタイムステップの数を少なくしてもいいかもしれません。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% 衛星質量
m = 1; % 1

% タイムステップ(s)
dt = 10;

% 時間 N×dt秒
N = 500;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%R = 0.01;
R = -1;

% 初期状態
s01 = [1/2; -1/(2*sqrt(3)); 0; 0; 0; 0];
s02 = [-1/2; -1/(2*sqrt(3)); 0; 0; 0; 0];
s03 = [0; 1/sqrt(3); 0; 0; 0; 0];
s0 = [s01; s02; s03]; % 6num×1

% 2衛星のそれぞれの目標状態
sd1 = [0; 1/sqrt(3); 0; 0; 0; 0];
sd2 = [1/2; -1/(2*sqrt(3)); 0; 0; 0; 0];
sd3 = [-1/2; -1/(2*sqrt(3)); 0; 0; 0; 0];
sd = [sd1; sd2; sd3];


% 衛星数
num = length(s0)/6;


%% Hill方程式 宇宙ステーション入門 P108

% 1衛星に関する状態方程式の係数行列
% x_dot = A_ x + B_ u
A_1 = [0, 0, 0, 1/2, 0, 0;
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

B_1 = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m]; % 6×3

%2衛星に関する状態方程式の係数行列
%A_ = [A_, zeros(6);zeros(6),A_];
%B_ = [B_,zeros(6,3);zeros(6,3),B_];

A_ = [];
B_ = [];

for i = 1:num
    A_ = blkdiag(A_, A_1);
    B_ = blkdiag(B_, B_1);
end

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
P = controllability_matrix(A_d, B_d, N); %6Nnum×3Nnum
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
Q = controllability_matrix2(A_d, N); %6N×6num

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

% 不等式制約2 (衛星間距離はR以下)
% ノミナルの状態プロファイルを設定
if not(R == -1)
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
    b2 = -R * calculate_norms(C2 * C1 * nominal_s) + create_matrix(C2 * C1 * nominal_s).' * C2 * C1 * Q * s0;

    A = [A1; A2];
    b = [b1; b2];
end

%% 等式制約

% 等式制約1 (運動量保存)
Aeq1 = [ones(N, 3*N*num), zeros(N, 1)];
beq1 = zeros(N, 1);

% 等式制約2 (最終状態固定)
Aeq2 = P(1:6*num,:);
beq2 = sd - Q(1:6*num,:) * s0;

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

%% 線形不等式制約線形計画問題 

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

disp('Objective function value:');
%disp(fval); % linprogのみ
disp("最大入力 u_max")
disp(x(3*num*N+1))


%% 図示

% 2衛星の動画を表示。
data = reorderMatrix(s);

% ビデオライターオブジェクトの作成
v = VideoWriter('points_motion.avi'); % AVIファイル形式で動画を保存
open(v);

% フィギュアの作成
figure;
axis equal;
xlim([-1, 1]); % xの範囲を調整
ylim([-1, 1]); % yの範囲を調整
hold on;

% 各フレームでの点の位置をプロットし、そのフレームを動画に書き込む
for i = 1:10*2:length(data)-1
    %disp(i)
    plot(data(i), data(i+1), 'o', 'MarkerSize', 10);
    plot(data(i+2), data(i+3), 'o', 'MarkerSize', 10);
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% ビデオの保存
close(v);

%% 関数リスト

% 関数の命名はテキトーです。すみません。

function mat = controllability_matrix(A, B, N)
    % 入力:
    % A: nxn の行列
    % B: nx1 のベクトル
    % N: 整数
    n = size(A, 1); % A行列の次元
    k = size(B, 2);
    mat = [];
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
        mat = [mat;mat_i];
    
    end

end

function mat = controllability_matrix2(A, N)
    mat = [];
    for j = 1:N
        mat_i = A^(N - j + 1);
        mat = [mat;mat_i];
    end

end

function matrix_3n_n = create_matrix(vec_3n)
    % 複数の相対位置ベクトルの内積をまとめて行うための行列を作る 3基以上の衛星の場合全通りの組み合わせをやる必要があるから面倒だな。
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

function B = reorderMatrix(A)
    idx = find(mod(1:length(A), 6) == 1 | mod(1:length(A), 6) == 2);
    A = flip(A(idx));
    B = [];
    %A = 1:100;
    n = 2;
    for i = 1:n:length(A)
        sub_vector = A(i:min(i+n-1, length(A)));
        B = [B; flip(sub_vector)];
    end
end
