%% 電磁力を用いた衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大磁気モーメントを最小化
% 進入禁止範囲rを設定
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

% シンボリック計算ツールボックスで求めた偏微分は入力に0を受け付けないから、初期値s01, s02に0をいれてはいけない。

% 1. scp_mpc.mを実行してスラスター衛星の場合を計算。
% 2. F2myu.mを実行して、スラスター入力を電流入力に変換。
% 3. これを実行。
% シミュレーションパラメータをscp_mpc.mと同じにする必要がある。

% 繰り返し最適化する場合は、このコードを何回も実行すればよい。

%% パラメータ設定

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% 衛星質量
m = 1; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 N×dt秒
N = 3;

% 衛星数
num = 2;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%R = 0.0;
r = -1;

% trust region
delta1 = 0.01;

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
A_ = [A_, zeros(6);zeros(6),A_];
B_ = [B_,zeros(6,3);zeros(6,3),B_];


% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

% 初期状態
s01 = [0.15; 0.0000001; 0.0000001; 0; 0; 0];
s02 = [-0.15; -0.0000001; -0.0000001; 0; 0; 0];
s0 = [s01; s02]; % 6num×1

% 2衛星のそれぞれの目標状態
%sd1 = [-0.5; 0.5; 0; 0; 0; 0];
%sd2 = [0.5; -0.5; 0; 0; 0; 0];

%レコード盤軌道
%√3rrが衛星間距離になることに注意
rr = sqrt(2)*0.925/2;

sd1 = [-2*rr*cos(n*N*dt); sqrt(3)*rr*sin(n*N*dt); rr*sin(n*N*dt); 2*n*rr*sin(n*N*dt); sqrt(3)*n*rr*cos(n*N*dt); n*rr*cos(n*N*dt)];
sd2 = [-2*rr*cos(n*N*dt + pi); sqrt(3)*rr*sin(n*N*dt + pi); rr*sin(n*N*dt + pi); 2*n*rr*sin(n*N*dt + pi); sqrt(3)*n*rr*cos(n*N*dt + pi); n*rr*cos(n*N*dt + pi)];
sd = [sd1; sd2];

% 微分式のセル
func_cell = create_func_cell();

% ノミナル軌道sによってPとQが変わる
A_list = create_A_list(num, N, s, s0, u_myu, A_d, B_d, u_myu_max, func_cell); % {A1, A2, ... ,AN}
B_list = create_B_list(num, N, s, s0, u_myu, B_d, u_myu_max, func_cell); % {B1, B2, ... ,BN}
C_list = create_C_list(num, N, s, s0, u_myu, B_d, u_myu_max, func_cell); % {C1, C2, ... ,CN}

A_mat = create_A_mat(A_list, num, N);
B_mat = create_B_mat(B_list, num, N);
A_mat2 = create_A_mat2(A_list, num, N);
C_mat = create_C_mat(C_list, num, N);

P = A_mat*B_mat; %6Nnum×3Nnum　
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1 
Q = A_mat2; 
R = A_mat*C_mat; 

disp("線形化したEMFFダイナミクスを用いて、軌道を再計算。ちゃんと目標値になっていたらok")
l_s = P * u_myu + Q * s0 + R;
disp(l_s(1:3))
disp(l_s(7:9))
% 最大磁気モーメント
u_myu_max = u_myu(3*N+1);
%% 評価関数

% 評価関数1(最大入力最小)
f1 = [zeros(1, 3*N), 1];  
f = f1;

%% 不等式制約

% 不等式制約1(全ての入力は最大入力以下)
% 最大入力との差が0より大きくなければならない。
A1 = [eye(3*N), -ones(3*N,1); -eye(3*N), -ones(3*N, 1)]; %6N×3Nnum+1
b1 = zeros(6*N, 1);%6Nnum×1

% 不等式制約2 (衛星間距離はr以上)
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

% 不等式制約3 (ノミナル軌道に対する変化量はδ以下 trust region)
% s - (PU + Qs0 + R) < δ
% -s + (PU + Qs0 + R) < δ

A3 = [-P; P];
b3 = [delta1 * ones(6*N*num, 1) - s + Q * s0 + R; delta1 * ones(6*N*num, 1) + s - Q * s0 - R];

% 不等式制約4 (磁気モーメントの変化量はδ2以下)
% U2 - U1 < δ
% U1 - U2 < δ
%A4 = [-[eye(N*num*3), zeros(N*num*3,1)]; [eye(N*num*3), zeros(N*num*3,1)]];
%b4 = [delta2 * ones(3*N*num, 1) - u_myu(1:end-1); delta2 * ones(3*N*num, 1) + u_myu(1:end-1)];


A = [A1; A2; A3];
b = [b1; b2; b3];

%% 等式制約

% 等式制約1 (運動量保存)
%Aeq1 = create_Aeq1(N, num);
%beq1 = zeros(3*N, 1);

% 等式制約2 (最終状態固定)
Aeq2 = P(1:6*num,:);
beq2 = sd - Q(1:6*num,:) * s0 - R(1:6*num,:);

Aeq = Aeq2;
beq = beq2;

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
s = P * x + Q * s0 + R;
u_myu = x;
disp("最大磁気モーメント u_myu_max()")
disp(x(3*N+1))
coilN = 100;
radius = 0.05;
I_max = 35;
mass = 1;
disp("最大電流 u_myu_max(A)/(coilN * pi * radius^2)")
disp(x(3*N+1)/(coilN * pi * radius^2));
I_max_list = [I_max_list; x(3*N+1)/(coilN * pi * radius^2)];


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
    B_mat = zeros(6*num*N, 3*N);
    for i = 1:N
        B_mat((i-1)*6*num+1:i*6*num, (i-1)*3+1:i*3) = B_list{N - i + 1};
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

function A_list = create_A_list(num, N, s, s0, myu1, A_d, B_d, myu_max_val, func_cell) % {A1, A2, ... ,AN}
    A_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        u1 = myu1(3*(N-i)+1:3*(N-i+1));
        Ak = create_Ak(A_d, B_d, sk, u1, num, myu_max_val, func_cell);
        A_list{i} = Ak;
    end
end

function B_list = create_B_list(num, N, s, s0, myu1, B_d, myu_max_val, func_cell) % {B1, B2, ... ,BN}
    B_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        u1 = myu1(3*(N-i)+1:3*(N-i+1));
        B = create_Bk(B_d, sk, num, myu_max_val, func_cell);
        B_list{i} = B;
    end
end

function C_list = create_C_list(num, N, s, s0, myu1, B_d, myu_max_val, func_cell) % {C1, C2, ... ,CN}
    C_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        u1 = myu1(3*(N-i)+1:3*(N-i+1));
        C = create_Ck(B_d, sk, u1, num, myu_max_val, func_cell);
        C_list{i} = C;
    end
end

function A = create_Ak(A_d, B_d, sk, myu1, num, myu_max_val, func_cell)
    A = zeros(6*num,6*num);
    dFds = dFds_func(sk, myu1, myu_max_val, func_cell);

    i = 1;
    A(6*(i-1)+1:6*i,:) = [A_d, zeros(6)] + B_d * dFds;

    i = 2;
    A(6*(i-1)+1:6*i,:) = [zeros(6), A_d] - B_d * dFds;
end

function B = create_Bk(B_d, sk, num, myu_max_val, func_cell)
    B = zeros(6*num,3);
    F = F_func(sk, myu_max_val, func_cell);

    i = 1;
    B(6*(i-1)+1:6*i,:) = B_d * F; 

    i = 2;
    B(6*(i-1)+1:6*i,:) = - B_d * F; 
end

function C = create_Ck(B_d, sk, myu1, num, myu_max_val, func_cell) 
    C = zeros(6*num,1);
    dFds = dFds_func(sk, myu1, myu_max_val, func_cell);
 
    i = 1;
    C(6*(i-1)+1:6*i,1) = - B_d * dFds * sk;

    i = 2;
    C(6*(i-1)+1:6*i,1) = B_d * dFds * sk;

end

function Aeq1 = create_Aeq1(N, num)
    matrix1 = [eye(3),eye(3)];
    Aeq1 = zeros(3*N, 3*num*N+1);
    for i = 1:N
        Aeq1(3*(i-1)+1:3*i, 3*num*(i-1)+1:3*num*i) = matrix1;
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

function func_cell = create_func_cell()
    syms x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max
     
    r = [x1 - x2; y1 - y2; z1 - z2];
    myu = [myu11; myu12; myu13];
    myu0 = 4*pi*1e-7; % 真空の透磁率
    
    %f = 3*myu0/(4*pi)*(dot(myu1, myu2)/norm(r)^5 * r + dot(myu1, r)/norm(r)^5 * myu2 + dot(myu2, r)/norm(r)^5 * myu1 - 5*dot(myu1, r)*dot(myu2, r)/norm(r)^7*r);
    f = 3*myu0*myu_max/(4*pi)*(1/norm(r)^4 * eye(3) - 3 * (r * r.')/norm(r)^6) * myu;
    F = 3*myu0*myu_max/(4*pi)*(1/norm(r)^4 * eye(3) - 3 * (r * r.')/norm(r)^6);

    df_dx1 = diff(f, x1);
    df_dy1 = diff(f, y1);
    df_dz1 = diff(f, z1);
    df_dx2 = diff(f, x2);
    df_dy2 = diff(f, y2);
    df_dz2 = diff(f, z2);
    df_dx1_func = matlabFunction(df_dx1, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    df_dy1_func = matlabFunction(df_dy1, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    df_dz1_func = matlabFunction(df_dz1, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    df_dx2_func = matlabFunction(df_dx2, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    df_dy2_func = matlabFunction(df_dy2, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    df_dz2_func = matlabFunction(df_dz2, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    f_func0 = matlabFunction(f, 'vars', [x1 y1 z1 x2 y2 z2 myu11 myu12 myu13 myu_max]);
    F_func0 = matlabFunction(F, 'vars', [x1 y1 z1 x2 y2 z2 myu_max]);
    func_cell = {df_dx1_func, df_dy1_func, df_dz1_func, df_dx2_func, df_dy2_func, df_dz2_func, f_func0, F_func0};
end


function dFds = dFds_func(s_val, myu, myu_max_val, func_cell)
    df_dx1_func = func_cell{1};
    df_dy1_func = func_cell{2};
    df_dz1_func = func_cell{3};
    df_dx2_func = func_cell{4};
    df_dy2_func = func_cell{5};
    df_dz2_func = func_cell{6};


    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3);
    x2_val = s_val(7);
    y2_val = s_val(8); 
    z2_val = s_val(9); 

    myu11_val = myu(1);
    myu12_val = myu(2);
    myu13_val = myu(3);
    df_dx1 = df_dx1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dy1 = df_dy1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dz1 = df_dz1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dx2 = df_dx2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dy2 = df_dy2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dz2 = df_dz2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    dFds = [df_dx1, df_dy1, df_dz1, zeros(3), df_dx2, df_dy2, df_dz2, zeros(3)]; % 3×12
end


function F = F_func(s_val, myu_max_val, func_cell)
    F_func0 = func_cell{8};
    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3); 
    x2_val = s_val(7);
    y2_val = s_val(8); 
    z2_val = s_val(9);
    F = F_func0(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu_max_val);
end


%{
function dFds1 = dFds1_func(s_val, myu, myu_max_val, func_cell)
    df_dx1_func = func_cell{1};
    df_dy1_func = func_cell{2};
    df_dz1_func = func_cell{3};

    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3);
    x2_val = s_val(7);
    y2_val = s_val(8); 
    z2_val = s_val(9); 

    myu11_val = myu(1);
    myu12_val = myu(2);
    myu13_val = myu(3);
    df_dx1 = df_dx1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dy1 = df_dy1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dz1 = df_dz1_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    dFds1 = [df_dx1, df_dy1, df_dz1, zeros(3)]; % 3×12
end

function dFds2 = dFds2_func(s_val, myu, myu_max_val, func_cell)
    df_dx2_func = func_cell{4};
    df_dy2_func = func_cell{5};
    df_dz2_func = func_cell{6};


    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3);
    x2_val = s_val(7);
    y2_val = s_val(8); 
    z2_val = s_val(9); 

    myu11_val = myu(1);
    myu12_val = myu(2);
    myu13_val = myu(3);
    df_dx2 = df_dx2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dy2 = df_dy2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    df_dz2 = df_dz2_func(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
    dFds2 = [df_dx2, df_dy2, df_dz2, zeros(3)]; % 3×12
end


function f = f_func(s_val, myu_val, myu_max_val, func_cell)
    f_func0 = func_cell{7};
    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3); 
    x2_val = s_val(7);
    y2_val = s_val(8); 
    z2_val = s_val(9);

    myu11_val = myu_val(1);
    myu12_val = myu_val(2);
    myu13_val = myu_val(3);
    f = f_func0(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu11_val, myu12_val, myu13_val, myu_max_val);
end
%}


%{
function dFdr = dFdr_func(r_val, myu, myu_max_val, func_cell)
    df_dxr_func = func_cell{1};
    df_dyr_func = func_cell{2};
    df_dzr_func = func_cell{3};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 
    df_dxr = df_dxr_func(xr_val, yr_val, zr_val, myu_max_val);
    df_dyr = df_dyr_func(xr_val, yr_val, zr_val, myu_max_val);
    df_dzr = df_dzr_func(xr_val, yr_val, zr_val, myu_max_val);
    dFdr = [df_dxr*myu, df_dyr*myu, df_dzr*myu]; % 3×3
end
%}