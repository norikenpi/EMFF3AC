%% num基の衛星のSCP-MPC（線形不等式制約線形計画問題 linprog）
% 最大入力を最小化
% 進入禁止範囲を設定可能
% 初めて実行する場合はR = -1にして進入禁止範囲制約を外してください。
% 2回目以降の実行は、既にある軌道をノミナル軌道としてSCP-MPCを行います。
% Daniel Morgan, et. al., “Spacecraft Swarm Guidance Using a Sequence of Decentralized Convex Optimizations”

%% パラメータ設定

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%d_avoid = 0.18;

%d_avoid = -1;


tic;



% 最終衛星間距離
d_target = 0.925;

scale = 10^(-6);

% 制御可能範囲(m)
d_max = 1.0122;
d_max = 1.0122*5;

% 衛星数　2基or5基or9基
num = 2;

% 衛星質量
m = 1; % 1
%m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 250;

%u_max = 1e-9;
coilN = 140;
radius = 0.05;
d_avoid = radius*6;
% 初期衛星間距離
d_initial = d_avoid/2;
s01 = [-d_initial; -d_initial; 0; 0; 0; 0];
s02 = [d_initial; d_initial; -0; 0; 0; 0];

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
     0, 0, 1/m]*scale; % 6×3

%2衛星に関する状態方程式の係数行
A_ = A;
B_ = B;
%{
for i = 2:num
    A_ = blkdiag(A_, A); % BにAを対角に追加
    B_ = blkdiag(B_, B); % BにAを対角に追加
end
%}

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

rr1 = d_target/4;
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];
%s0 = adjust_cog([s01, s02], num); % 6num×1
s0 = s01;


% 各時刻の状態←各時刻の入力プロファイル,初期状態
% S = PU + Qs_0
P = create_P(A_d, B_d, N); %6Nnum×3Nnum
%P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
Q = create_Q(A_d, N); %6N×6num

%% 等式制約

% 等式制約1 (運動量保存)

%Aeq1 = create_Aeq1(N, num);
%beq1 = zeros(3*N, 1);

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
%Aeq2 = mat * relative_mat * P(1:6*num,:);
%beq2 = zeros(4,1) - mat * relative_mat * Q(1:6*num,:) * s0;


%Aeq = Aeq1;
%beq = beq1;
d_max_list = d_max * ones(N, 1);
%% 線形不等式制約線形計画問題 
time = toc
tic;;kl
[x, fval, exitflag, output] = solveOptimizationProblem(n, myu_list, N, myu_max, P, Q, s0, d_avoid, A_d, B_d);
time = toc
% 衛星の状態
u_list = myu_list2u_list(myu_list, N, s0, myu_max, A_d, B_d);
s = P * u_list + Q * s0;
s1 = s;
myu = x;

disp("最大磁気モーメント myu_max")
disp(max(abs(x))*scale)

%disp("最大電力")
%disp((max(vecnorm(u_mat,2,1))*scale)^2)

disp("安定チェック")
error = mat * s(1:6);
disp(error)

disp("最小化したい評価値")
disp(sum(abs(error)))
%% 図示

plot_s(s, num, N, rr, d_target)

%% 関数リスト

function u_list = myu_list2u_list(myu_list, N, s0, myu_max, A_d, B_d)
    u_list = zeros(3*N, 1);
    func_cell = create_func_cell();
    
    s_minus = s0;

    for i = 1:N
        myu00 = myu_list(3*N - 3*(i-1) - 2:3*N - 3*(i-1));
        F0 = F_func(s_minus, myu_max, func_cell)*myu00;
        u_list(3*N - 3*(i-1) - 2:3*N - 3*(i-1)) = F0;
        s_minus = A_d * s_minus + B_d * F0;
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

function F = F_func(s_val, myu_max_val, func_cell)
    F_func0 = func_cell{8};
    x1_val = s_val(1);
    y1_val = s_val(2); 
    z1_val = s_val(3); 
    x2_val = -s_val(1);
    y2_val = -s_val(2); 
    z2_val = -s_val(3);
    F = F_func0(x1_val, y1_val, z1_val, x2_val, y2_val, z2_val, myu_max_val);
end
function [x, fval, exitflag, output] = solveOptimizationProblem(n, x0, N, myu_max, P, Q, s0, d_avoid, A_d, B_d)
    % 初期化
    A = [];  % 不等式制約 A*x <= b の A
    b = [];  % 不等式制約 A*x <= b の b
    Aeq = [];
    beq = []; 
    lb = []; % 変数の下限
    ub = []; % 変数の上限

    % オプションの設定
    options = optimoptions('fmincon', ...
                       'Algorithm', 'interior-point', ...
                       'TolFun', 1e-6, ...
                       'TolX', 1e-6, ...
                       'TolCon', 1e-6, ...
                       'Display', 'iter', ...
                       'MaxIterations', 400, ...
                       'StepTolerance', 1e-6);

    options = optimoptions('fmincon', ...
                       'Algorithm', 'interior-point', ...
                       'TolFun', 1e-6, ...
                       'TolX', 1e-6, ...
                       'TolCon', 1e-6, ...
                       'MaxIterations', 1, ...
                       'Display', 'iter-detailed');
    
    fun =  @(x) objectiveFunction(n, x, P, Q, s0, N, myu_max, A_d, B_d);

    c_ceq = @(x) nonlinearConstraints(x, N, myu_max, P, Q, s0, d_avoid, A_d, B_d);
    % fminconの呼び出し
    [x, fval, exitflag, output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, c_ceq, options);
end

function f = objectiveFunction(n, x, P, Q, s0, N, myu_max, A_d, B_d)
    % 目的関数の計算
    kA = 2e-3;
    thetaP = pi/6;
    %relative_mat = [eye(6),-eye(6)];
    mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
           2,0,0,0,1/n,0;
           0,0,0,-1/(n*tan(thetaP)),0,1/n;
           -1/(n*tan(thetaP)),0,1/n, 0,0,0];
    u_list = myu_list2u_list(x, N, s0, myu_max, A_d, B_d);
    
    f = sum(abs(mat * (P(1:6,:) * u_list + Q(1:6,:) * s0))); % xを用いた計算
end

function [c, ceq] = nonlinearConstraints(x, N, myu_max, P, Q, s0, d_avoid, A_d, B_d)
    
    % 非線形制約の計算
    % 発電量拘束
    
    P_list = zeros(N,1);
    for i = 1:N
        P_list(i) = norm(x(3*(i-1)+1:3*i))^2 - myu_max^2;   
    end
    

    % 進入禁止制約
    dist_margin_list = zeros(N,1);
    u_list = myu_list2u_list(x, N, s0, myu_max, A_d, B_d);
    s = P * u_list + Q * s0;
    for i = 1:N
        dist_margin_list(i) = d_avoid/2 - norm(s(6*(i-1)+1:6*(i-1)+3));   
    end

    % 制御可能範囲制約
    %%%%%%%%%%%%%%%%%%%
    
    c = [P_list; dist_margin_list];  % 不等式制約 c(x) <= 0 進入禁止制約、発電量拘束90-iop[k
    c = P_list;
    ceq = [];% 等式制約 ceq(x) = 0
end
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
    satellites = cell(1, 2);

    % 衛星インスタンス生成
    for i = 1:num
        satellites{i} = zeros(N, 3);
    end
    
    % 衛星データ格納
    for i = 1:N
        for j = 1:1 % for j = 1:num
            satellites{j}(i,:) = data(3*(i-1)+3*(j-1)+1:3*(i-1)+3*(j-1)+3).';
        end
    end

    satellites{2} = -satellites{1}; 
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

