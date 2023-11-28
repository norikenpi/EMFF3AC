%% 1基の原点距離に反比例した力を出力できる衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% 計算にだいぶ時間がかかるので、もっとタイムステップの数を少なくしてもいいかもしれません。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”


% ダイナミクスは原点距離を利用


% scp_mpc1.mを実行してスラスター衛星の場合を計算
% F2I.mを実行して、原点距離の4乗に反比例した出力が得られるスラスターに変更
% これを実行。

% 入力の単位をμにすることでスケールを合わせていることに注意
%% パラメータ設定

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% 衛星質量
m = 1; % 1

% タイムステップ(s)
dt = 10;

% 時間 N×dt秒
N = 1000;

% 衛星数
num = 2;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
r = 0.01 ;
%R = -1;

delta = 0.04;
%s = s1;

%% Hill方程式 宇宙ステーション入門 P108

% 1衛星に関する状態方程式の係数行列
% x_dot = A_ x + B_ u
A_ = [0, 0, 0, 1/2, 0, 0;
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

B_ = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m]; % 6×3

%2衛星に関する状態方程式の係数行列
% A_ = [A_, zeros(6);zeros(6),A_];
% B_ = [B_,zeros(6,3);zeros(6,3),B_];


% 1衛星に関する離散時間状態方程式の係数行列
A_d = eye(6) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

% 初期状態
s01 = [0.5; 0; 0; 0; 0; 0];
s02 = [-0.5; 0; 0; 0; 0; 0];
s0 = [s01; s02]; % 6num×1

% 2衛星のそれぞれの目標状態
sd1 = [0; 0.5; 0; 0; 0; 0];
sd2 = [0; -0.5; 0; 0; 0; 0];
sd = [sd1; sd2];

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
%P = controllability_matrix(A_d, B_d, N); %6Nnum×3Nnum
%P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
%Q = controllability_matrix2(A_d, N); %6N×6num

% ノミナル軌道sによってPとQが変わる
A_list = create_A_list(num, N, s, s0, u_I, A_d, B_d); % {A1, A2, ... ,AN}
B_list = create_B_list(num, N, s, s0, B_d); % {B1, B2, ... ,BN}
C_list = create_C_list(num, N, s, s0, u_I, B_d); % {C1, C2, ... ,CN}

A_mat = create_A_mat(A_list, num, N);
B_mat = create_B_mat(B_list, num, N);
A_mat2 = create_A_mat2(A_list, num, N);
C_mat = create_C_mat(C_list, num, N);

P = A_mat*B_mat; %6Nnum×3Nnum　
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1 
Q = A_mat2; 
R = A_mat*C_mat; 

% ノミナル起動をもとにP, Q, Rを再計算しなおしているが、入力は前のまま。
% つまり、trust region が大きすぎると以下を計算しても最終値が合わない。
disp("linearize_error")
l_s = P * u_I + Q * s0 + R;
disp(l_s(1:3))

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
%{
matrix01 = [ones(1, 3), zeros(1, 3)];
A2_ = matrix01;
for i = 2:N
    A2_ = blkdiag(A2_, matrix01);
end
A2 = - A2_*P;

b2 = - r * ones(N, 1) + A2_ * Q * s0 + A2_ * R;
%}

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
b2 = -r * calculate_norms(C2 * C1 * nominal_s) + create_matrix(C2 * C1 * nominal_s).' * C2 * C1 * (Q * s0 + R);

% 不等式制約3 (ノミナル軌道に対する変化量はδ trust region)
% s - (PU + Qs0 + R) < δ
% -s + (PU + Qs0 + R) < δ

A3 = [-P; P];
b3 = [delta * ones(6*N*num, 1) - s + Q * s0 + R; delta * ones(6*N*num, 1) + s - Q * s0 - R];

A = [A1; A2; A3];
b = [b1; b2; b3];

%% 等式制約

% 等式制約1 (運動量保存)
Aeq1 = [ones(N, 3*N*num), zeros(N, 1)];
beq1 = zeros(N, 1);

% 等式制約2 (最終状態固定)
Aeq2 = P(1:6*num,:);
beq2 = sd - Q(1:6*num,:) * s0 - R(1:6*num,:);

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];

%% 線形不等式制約線形計画問題 
% 解はnum×N×3自由度

% linprogを使う場合
%{
[x,fval,exitflag,output,lambda] = ...
   linprog(f, A, b, Aeq, beq);
%}

% cvxを使う場合

cvx_begin sdp quiet
    variable x(size(f, 2))
    minimize(f * x)
    subject to
        A * x <= b;
        Aeq * x == beq;
cvx_end


% 衛星の状態
s = P * x + Q * s0 + R;
u_I = x;
disp('Objective function value:');
%disp(fval); % linprogのみ
disp("最大入力 u_max")
disp(x(3*num*N+1) * 10^(-6))


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
function A_mat = create_A_mat(A_list, num, N)
    A_mat = zeros(6*num*N);
    for k = 1:N % 行
        for i = 1:N  % 列
            if i <= k-1
                mat = zeros(6*num);
            elseif i == k
                mat = eye(6*num);
            elseif k+1 <= i
                mat = eye(6*num);
                for j = (N-i+2):(N-k+1)
                    mat = A_list{j}*mat;
                end
            end
            A_mat((k-1)*6*num+1:k*6*num, (i-1)*6*num+1:i*6*num) = mat;
        end
    end
end

function B_mat = create_B_mat(B_list, num, N)
    B_mat = zeros(6*num*N, 3*num*N);
    for i = 1:N
        B_mat((i-1)*6*num+1:i*6*num, (i-1)*3*num+1:i*3*num) = B_list{N - i + 1};
    end
end

function A_mat2 = create_A_mat2(A_list, num, N)
    A_mat2 = zeros(6*num*N, 6*num);
    for i = 1:N
        mat = eye(6*num);
        for j = 1:(N-i+1)
            mat = A_list{j} * mat;
        end
        A_mat2((i-1)*6*num+1:i*6*num, :) = mat;
    end
end

function C_mat = create_C_mat(C_list, num, N)
    C_mat = zeros(6*num*N, 1);
    for i = 1:N
        C_mat((i-1)*6*num+1:i*6*num, :) = C_list{N-i+1};
    end
end

function A_list = create_A_list(num, N, s, s0, u, A_d, B_d) % {A1, A2, ... ,AN}
    A_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        uk = u(3*(N-i)*num+1:3*(N-i+1)*num);
        Ak = create_Ak(A_d, B_d, sk, uk, num);
        A_list{i} = Ak;
    end
end

function B_list = create_B_list(num, N, s, s0, B_d) % {B1, B2, ... ,BN}
    B_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        B = create_Bk(B_d, sk, num);
        B_list{i} = B;
    end
end

function C_list = create_C_list(num, N, s, s0, u, B_d) % {C1, C2, ... ,CN}
    C_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        uk = u(3*(N-i)*num+1:3*(N-i+1)*num);
        C = create_Ck(B_d, sk, uk, num);
        C_list{i} = C;
    end
end

function A = create_Ak(A_d, B_d, sk, uk, num)
    A = zeros(6*num,6*num);
    for i = 1:num 
        xk_i = sk(6*(i-1)+1:6*(i-1)+3);
        uk_i = uk(3*(i-1)+1:3*i);
        dfds = -4*norm(xk_i)^(-6)*uk_i*[xk_i.', zeros(1,3)] * 10^(-6);
        dfds_m = B_d * dfds;
        A(6*(i-1)+1:6*i,6*(i-1)+1:6*i) = A_d + dfds_m;
    end
end

function B = create_Bk(B_d, sk, num)
    B = zeros(6*num,3*num);
    for i = 1:num 
        xk_i = sk(6*(i-1)+1:6*(i-1)+3);
        dfdu = eye(3)/norm(xk_i)^4 * 10^(-6);
        B(6*(i-1)+1:6*i,3*(i-1)+1:3*i) = B_d * dfdu; 
    end
end

function C = create_Ck(B_d, sk, uk, num) 
    C = zeros(6*num,1);
    for i = 1:num 
        sk_i = sk(6*(i-1)+1:6*i);
        xk_i = sk(6*(i-1)+1:6*(i-1)+3);
        uk_i = uk(3*(i-1)+1:3*i);
        f = uk_i/norm(xk_i)^(4) * 10^(-6);
        dfds = -4*norm(xk_i)^(-6)*uk_i*[xk_i.', zeros(1,3)] * 10^(-6);
        dfdu = eye(3)/norm(xk_i)^4 * 10^(-6);
        C(6*(i-1)+1:6*i,1) = B_d * (f - dfds * sk_i - dfdu * uk_i);
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














%{








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

function mat = controllability_matrix3(A_, B_, N, s)
    % 入力:
    % A: nxn の行列
    % B: nx1 のベクトル
    % N: タイムステップの数
    n = size(A_, 1); % A行列の行数
    k = size(B_, 2); % B行列の列数
    mat = [];
    A_list = cell(1, N);
    B_list = cell(1, N);
    for i = 1:N
        sk = s(6*(i-1)+1:6*i);
        xk = x(3*(i-1)+1:3*i);
        A = create_Ak(A_, sk, xk);
        B = create_Bk(B_, sk);
        A_list{i} = A;
        B_list{i} = B;
    end
    
    for j = 1:N % 行
        for i = 1:N % 列（下から）
            if i < j
                mat_i(:, k*(i-1)+1: k*i) = zeros(n, k);
            else
                % Aは累乗じゃなくて違うAをかけないといけない。
                % i行j列目の係数
                AA = A_list{N-i+1};
                for k = (N-i+1):(N-j)
                    AA = AA * A_list{k};
                end
                mat_i(:, k*(i-1)+1: k*i) = AA * B_list{N-j+1};
                % mat_i(:, k*(i-1)+1: k*i) = A^(i-j) * B;
            end
            
        end
        mat = [mat;mat_i];
    end
end

function mat = controllability_matrix4(A_, N, s, x)
    mat = [];
    A_list = cell(1, N);
    for i = 1:N
        sk = s(6*(i-1)+1:6*i);
        xk = x(3*(i-1)+1:3*i);
        A = create_Ak(A_, sk, xk);
        A_list{i} = A;
    end
    for j = 1:N
        AA = A_list{j};
        for k = j+1:N
            AA = AA * A_list{k};
        end
        mat_i = AA;
        mat = [mat;mat_i];
    end
end

function mat = controllability_matrix5(A_, N, s, x)
    % Aで構成された行列とCで構成された行列を掛け合わせる感じにしたい。
end

function A_mat = create_A_mat(A_list)
    for j = 1:N % 行
        for i = 1:N % 列（下から）
            if i < j
                mat_i(:, k*(i-1)+1: k*i) = zeros(n, k);
            else
                % Aは累乗じゃなくて違うAをかけないといけない。
                % i行j列目の係数
                AA = A_list{N-i+1};
                for k = (N-i+1):(N-j)
                    AA = AA * A_list{k};
                end
                mat_i(:, k*(i-1)+1: k*i) = AA * B_list{N-j+1};
                % mat_i(:, k*(i-1)+1: k*i) = A^(i-j) * B;
            end
            
        end
        A_mat = [mat;mat_i];
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


%}