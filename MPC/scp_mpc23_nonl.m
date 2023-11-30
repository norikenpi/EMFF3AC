%% 電磁力を用いた衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大磁気モーメントを最小化
% 進入禁止範囲rを設定
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% 計算にだいぶ時間がかかるので、もっとタイムステップの数を少なくしてもいいかもしれません。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”



% シンボリック計算ツールボックスで求めた偏微分は入力に0を受け付けないから、初期値に0をいれてはいけない。


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
dt = 1;

% 時間 N×dt秒
N = 500;

% 衛星数(2以外動かない)
num = 2;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
r = 0.01 ;

delta = 0.001;
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
s01 = [0.5; 0.0000001; 0.0000001; 0; 0; 0];
s02 = [-0.5; -0.0000001; -0.0000001; 0; 0; 0];
s0 = [s01; s02]; % 6num×1

% 2衛星のそれぞれの目標状態
sd1 = [-0; 0.5; 0; 0; 0; 0];
sd2 = [0; -0.5; 0; 0; 0; 0];
sd = [sd1; sd2];

% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
%P = controllability_matrix(A_d, B_d, N); %6Nnum×3Nnum
%P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
%Q = controllability_matrix2(A_d, N); %6N×6num

func_cell = create_func_cell();

% ノミナル軌道sによってPとQが変わる
A_list = create_A_list(num, N, s, s0, u_myu, A_d, B_d, func_cell); % {A1, A2, ... ,AN}
B_list = create_B_list(num, N, s, s0, u_myu, B_d, func_cell); % {B1, B2, ... ,BN}
C_list = create_C_list(num, N, s, s0, u_myu, B_d, func_cell); % {C1, C2, ... ,CN}

A_mat = create_A_mat(A_list, num, N);
B_mat = create_B_mat(B_list, num, N);
A_mat2 = create_A_mat2(A_list, num, N);
C_mat = create_C_mat(C_list, num, N);

P = A_mat*B_mat; %6Nnum×3Nnum　
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1 
Q = A_mat2; 
R = A_mat*C_mat; 

disp("線形化したダイナミクスを用いて、軌道を再計算。ちゃんと目標値になっていたらok")
l_s = P * u_myu + Q * s0 + R;
disp(l_s(1:3))
disp(l_s(7:9))

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

% 不等式制約2 (衛星間距離はr以下)
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
disp('Objective function value:');
%disp(fval); % linprogのみ
disp("最大磁気モーメント u_myu_max(A)")
disp(x(3*num*N+1))
coilN = 1000;
radius = 0.05;
I_max = 20;
mass = 1;
disp("最大電流 u_myu_max(A)/(coilN * pi * radius^2)")
disp(x(3*num*N+1)/(coilN * pi * radius^2));


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

function A_list = create_A_list(num, N, s, s0, u, A_d, B_d, func_cell) % {A1, A2, ... ,AN}
    A_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        uk = u(3*(N-i)*num+1:3*(N-i+1)*num);
        Ak = create_Ak(A_d, B_d, sk, uk, num, func_cell);
        A_list{i} = Ak;
    end
end

function B_list = create_B_list(num, N, s, s0, u, B_d, func_cell) % {B1, B2, ... ,BN}
    B_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        uk = u(3*(N-i)*num+1:3*(N-i+1)*num);
        B = create_Bk(B_d, sk, uk, num, func_cell);
        B_list{i} = B;
    end
end

function C_list = create_C_list(num, N, s, s0, u, B_d, func_cell) % {C1, C2, ... ,CN}
    C_list = cell(1, N);
    for i = 1:N
        if i == 1
            sk = s0;
        else 
            sk = s(6*(N-i+1)*num+1:6*(N-i+2)*num);
        end
        uk = u(3*(N-i)*num+1:3*(N-i+1)*num);
        C = create_Ck(B_d, sk, uk, num, func_cell);
        C_list{i} = C;
    end
end

function A = create_Ak(A_d, B_d, sk, uk, num, func_cell)
    A = zeros(6*num,6*num);
    sk_r = [sk(1:6) - sk(7:12); sk(7:12) - sk(1:6)]; 
    for i = 1:num 
        xk_ri = sk_r(6*(i-1)+1:6*(i-1)+3);
        if i == 1
            uk_1 = uk(1:3);
            uk_2 = uk(4:6);
        elseif i == 2
            uk_2 = uk(1:3);
            uk_1 = uk(4:6);
        end
        dfds = dfdr_func(xk_ri, uk_1, uk_2, func_cell);
        dfds = [dfds, zeros(3,3)];
        dfds_m = B_d * dfds;
        A(6*(i-1)+1:6*i,6*(i-1)+1:6*i) = A_d + dfds_m;
    end
end

function B = create_Bk(B_d, sk, uk, num, func_cell)
    B = zeros(6*num,3*num);
    sk_r = [sk(1:6) - sk(7:12); sk(7:12) - sk(1:6)]; 
    for i = 1:num 
        xk_ri = sk_r(6*(i-1)+1:6*(i-1)+3);
        if i == 1
            uk_1 = uk(1:3);
            uk_2 = uk(4:6);
        elseif i == 2
            uk_2 = uk(1:3);
            uk_1 = uk(4:6);
        end
        dfdu = dfdmyu1_func(xk_ri, uk_1, uk_2, func_cell);
        B(6*(i-1)+1:6*i,3*(i-1)+1:3*i) = B_d * dfdu; 
    end
end

function C = create_Ck(B_d, sk, uk, num, func_cell) 
    C = zeros(6*num,1);
    sk_r = [sk(1:6) - sk(7:12); sk(7:12) - sk(1:6)]; 
    for i = 1:num 
        sk_i = sk(6*(i-1)+1:6*i);
        xk_ri = sk_r(6*(i-1)+1:6*(i-1)+3);
        uk_i = uk(3*(i-1)+1:3*i);

        if i == 1
            uk_1 = uk(1:3);
            uk_2 = uk(4:6);
        elseif i == 2
            uk_2 = uk(1:3);
            uk_1 = uk(4:6);
        end
        f = f_func(xk_ri, uk_1, uk_2, func_cell);
        dfds = dfdr_func(xk_ri, uk_1, uk_2, func_cell);
        dfds = [dfds, zeros(3,3)];
        dfdu = dfdmyu1_func(xk_ri, uk_1, uk_2, func_cell);
        C(6*(i-1)+1:6*i,1) = B_d * (f - dfds * sk_i - dfdu * uk_i);
    end
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
    syms xr yr zr myu11 myu12 myu13 myu21 myu22 myu23
    
    r = [xr; yr; zr];
    myu1 = [myu11; myu12; myu13];
    myu2 = [myu21; myu22; myu23];
    myu0 = 4*pi*1e-7; % 真空の透磁率
    
    f = 3*myu0/(4*pi)*(dot(myu1, myu2)/norm(r)^5 * r + dot(myu1, r)/norm(r)^5 * myu2 + dot(myu2, r)/norm(r)^5 * myu1 - 5*dot(myu1, r)*dot(myu2, r)/norm(r)^7*r);
    
    df_dxr = diff(f, xr);
    df_dyr = diff(f, yr);
    df_dzr = diff(f, zr);
    df_dmyu11 = diff(f, myu11);
    df_dmyu12 = diff(f, myu12);
    df_dmyu13 = diff(f, myu13);
    df_dmyu21 = diff(f, myu21);
    df_dmyu22 = diff(f, myu22);
    df_dmyu23 = diff(f, myu23);

    df_dxr_func = matlabFunction(df_dxr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dyr_func = matlabFunction(df_dyr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dzr_func = matlabFunction(df_dzr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu11_func = matlabFunction(df_dmyu11, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu12_func = matlabFunction(df_dmyu12, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu13_func = matlabFunction(df_dmyu13, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu21_func = matlabFunction(df_dmyu21, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu22_func = matlabFunction(df_dmyu22, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    df_dmyu23_func = matlabFunction(df_dmyu23, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    f_func0 = matlabFunction(f, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    
    
    func_cell = {df_dxr_func, df_dyr_func, df_dzr_func, df_dmyu11_func, df_dmyu12_func, df_dmyu13_func, df_dmyu21_func, df_dmyu22_func, df_dmyu23_func, f_func0};
end

function dfdr = dfdr_func(r_val, myu1_val, myu2_val, func_cell)
    df_dxr_func = func_cell{1};
    df_dyr_func = func_cell{2};
    df_dzr_func = func_cell{3};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dxr = df_dxr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dyr = df_dyr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dzr = df_dzr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);

    dfdr = [df_dxr, df_dyr, df_dzr]; % 3×3
end

function dfdmyu1 = dfdmyu1_func(r_val, myu1_val, myu2_val, func_cell)
    df_dmyu11_func = func_cell{4};
    df_dmyu12_func = func_cell{5};
    df_dmyu13_func = func_cell{6};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dmyu1x = df_dmyu11_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu1y = df_dmyu12_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu1z = df_dmyu13_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);

    dfdmyu1 = [df_dmyu1x, df_dmyu1y, df_dmyu1z]; % 3×3
end

function dfdmyu2 = dfdmyu2_func(r_val, myu1_val, myu2_val, func_cell)
    df_dmyu21_func = func_cell{7};
    df_dmyu22_func = func_cell{8};
    df_dmyu23_func = func_cell{9};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dmyu2x = df_dmyu21_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu2y = df_dmyu22_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu2z = df_dmyu23_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);

    dfdmyu2 = [df_dmyu2x, df_dmyu2y, df_dmyu2z]; % 3×3    
end

function F = f_func(r_val, myu1_val, myu2_val, func_cell)
    f_func0 = func_cell{10};
    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    F = f_func0(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
end
