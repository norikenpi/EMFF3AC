%% num基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定可能
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定
% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）


%これを外すと以下のエラーが出る。
%cvx から double に変換できません。
clearvars myu_mat
clearvars s_mat

% 最終衛星間距離
d_target = 0.925;

% 衛星数　2基or5基or9基
num = 2;

% 衛星質量
m = 1; % 1
%m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 10;

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
disp("最大電流設定")
disp(I_max)
disp("最大磁気モーメント設定")
disp(myu_max)

u_myu =  myu_list;
d_avoid = radius*6;
% 初期衛星間距離
d_initial = d_avoid/2;
s01 = [-d_initial; -d_initial; 0.00005; 0; 0; 0];
s02 = [d_initial; d_initial; -0.00005; 0; 0; 0];

delta_r = 300;
delta_myu = myu_max*2000;

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
     0, 0, 1/m]; % 6×3

%2衛星に関する状態方程式の係数行
A_ = A;
B_ = B;

for i = 2:num
    A_ = blkdiag(A_, A); % BにAを対角に追加
    B_ = [B_; B]; % BにAを対角に追加
end

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

rr1 = d_target/4;
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];
s0 = adjust_cog([-s01, -s02], num); % 6num×1

% 微分式のセル
func_cell = create_func_cell();
%u_myu = myu_list;

% ノミナル軌道sによってPとQが変わる
A_list = create_A_list(num, N, s, s0, u_myu, A_d, B_d, myu_max, func_cell); % {A1, A2, ... ,AN}
B_list = create_B_list(num, N, s, s0, u_myu, B_d, myu_max, func_cell); % {B1, B2, ... ,BN}
C_list = create_C_list(num, N, s, s0, u_myu, B_d, myu_max, func_cell); % {C1, C2, ... ,CN}

A_mat = create_A_mat(A_list, num, N);
B_mat = create_B_mat(B_list, num, N);
A_mat2 = create_A_mat2(A_list, num, N);
C_mat = create_C_mat(C_list, num, N);

P = A_mat*B_mat; %6Nnum×3Nnum
Q = A_mat2; 
R = A_mat*C_mat; 
%{
format long
disp("1ステップ検証")
F = F_func(s0, myu_max, func_cell);
s11 = A_d * s0 + B_d*F*u_myu(3*N-2:3*N);
disp(s11)
F = F_func(s11, myu_max, func_cell);
s12 = A_d * s11 + B_d*F*u_myu(3*N-5:3*N-3);
disp(s12)
F = F_func(s12, myu_max, func_cell);
s13 = A_d * s12 + B_d*F*u_myu(3*N-8:3*N-6);
disp(s13)
F = F_func(s13, myu_max, func_cell);
s14 = A_d * s13 + B_d*F*u_myu(3*N-11:3*N-9);
disp(s14)
%}
disp("線形化したEMFFダイナミクスを用いて、軌道を再計算。ちゃんと目標値になっていたらok")
l_s = P * u_myu + Q * s0 + R;
disp(l_s(1:3))
disp(l_s(7:9))


%% 評価関数

%% 不等式制約

% 不等式制約1(全ての入力は最大入力以下)
% 最大入力との差が0より大きくなければならない。
%A1 = [eye(3*N*num), -ones(3*N*num,1); -eye(3*N*num), -ones(3*N*num, 1)]; %6N×3Nnum+1
%b1 = zeros(6*N*num, 1);%6Nnum×1


% 不等式制約2 (衛星間距離はd_avoid以上)
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
b2 = -d_avoid * calculate_norms(C2 * C1 * nominal_s) + create_matrix(C2 * C1 * nominal_s).' * C2 * C1 * (Q * s0 + R);
% 不等式制約3 (ノミナル軌道に対する変化量はδ以下 trust region)
% s - (PU + Qs0 + R) < δ
% -s + (PU + Qs0 + R) < δ

A3 = [-P; P];
b3 = [delta_r * ones(6*N*num, 1) - s + Q * s0 + R; delta_r * ones(6*N*num, 1) + s - Q * s0 - R];

% 不等式制約4 (磁気モーメントの変化量はδ2以下)
% U2 - U1 < δ
% U1 - U2 < δ
A4 = [-eye(N*3); eye(N*3)];
b4 = [delta_myu * ones(3*N, 1) - u_myu; delta_myu * ones(3*N, 1) + u_myu];
%A = [A2; A3; A4];
%b = [b2; b3; b4];
A = [A2; A4];
b = [b2; b4];

%% 等式制約


%% 線形不等式制約線形計画問題 

kA = 2e-3;
thetaP = pi/6;
relative_mat = [eye(6),-eye(6)];
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];

% 解はnum×N×3自由度
cvx_begin
    variable x(3*N)
    minimize(sum(abs(mat * (P(1:6,:) * x + Q(1:6,:) * s0 + R(1:6,:)))))

    %pos = P * x + Q * s0;

    subject to
        % 不等式制約
        % 進入禁止制約
        % 位置trust region
        % 磁気モーメントtrust region
        A * x <= b;

        % 太陽光パネルの発電量拘束
        for i = 1:N
            myu_mat(:,i) = x(3*(i-1)+1:3*i);
            
            [myu_max^2, x(3*(i-1)+1:3*i).';
            x(3*(i-1)+1:3*i), eye(3)] >= 0;

        end
        %{
        [myu_max^2*eye(N), myu_mat.';
         myu_mat, eye(3)] >= 0;
        %}   
cvx_end
cvx_status


% 衛星の状態
s = P * x + Q * s0 + R;
s1 = s;
u = x;

disp("最大磁気モーメント myu_max")
disp(max(vecnorm(myu_mat,2,1)))

disp("最大電力")
disp(R_rho*(max(vecnorm(myu_mat,2,1))/(coilN * radius^2 * pi))^2)

disp("安定チェック")
x_r = s(1:6)-s(7:12);
error = mat * x_r;
disp(error)

disp("最小化したい評価値")
disp(sum(abs(error)))
%% 図示

plot_s(s, num, N, rr, d_target)

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
    A(6*(i-1)+1:6*i,:) = A_d(1:6,:) + B_d(1:6,:) * dFds;

    i = 2;
    A(6*(i-1)+1:6*i,:) = A_d(7:12,:) - B_d(7:12,:) * dFds;
end

function B = create_Bk(B_d, sk, num, myu_max_val, func_cell)
    B = zeros(6*num,3);
    F = F_func(sk, myu_max_val, func_cell);

    i = 1;
    B(6*(i-1)+1:6*i,:) = B_d(1:6,:) * F; 

    i = 2;
    B(6*(i-1)+1:6*i,:) = - B_d(7:12,:) * F; 
end

function C = create_Ck(B_d, sk, myu1, num, myu_max_val, func_cell) 
    C = zeros(6*num,1);
    dFds = dFds_func(sk, myu1, myu_max_val, func_cell);
 
    i = 1;
    C(6*(i-1)+1:6*i,1) = - B_d(1:6,:) * dFds * sk;

    i = 2;
    C(6*(i-1)+1:6*i,1) = B_d(7:12,:) * dFds * sk;

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

function s = adjust_cog(s_mat, num)
    
    cog = sum(s_mat, 2)/num;
    s = zeros(6*num, 1);
    for i = 1:num
        s(6*(i-1)+1: 6*i) = s_mat(:,i) - cog;
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

