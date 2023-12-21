%satteliteというcell 配列を入れてそこに位置を記録していこう。
%高橋座標系になっていることに注意。

num = 1;
dt = 10;
N = 10;
n = 0.0011; % 0.0011
m = 1; % 1
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
% 初期衛星間距離
d_initial = d_avoid/2;

s0 = [d_initial; d_initial; -0.00005; 0; 0; 0];

u_list = zeros(3*N,1);
myu_list = zeros(3*N,1);
myu_list2 = zeros(3*N,1);

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



d_target = 0.925;

rr1 = d_target/(2*sqrt(3));
rr2 = sqrt(2)*d_target/2;

rr = [rr1,rr2];

state = zeros(N,6);
state(1,:) = s0.';
s = zeros(6*N*2,1);
thetaP = pi/6;
rd = 0;

for i = 1:N
    X = state(i,:).';
    if norm(X(1:3)) > d_avoid/2
        C110 = coord2const(X, n);
        kA = 2;%2e-3;
        kB = 1;%1e-3;
        C1 = C110(1); C4 = C110(4); C5 = C110(5);
        C2 = C110(2); C3 = C110(3);
        r_xy = C110(7); phi_xy = C110(8); %phi_xy = atan2(o_r_ji(1),o_r_ji(2)/2);
        C4d = 3*n*C1/kA; %目標値
        dC4 = C4-C4d; %C4偏差
        C5d = C2/tan(thetaP);
        u_A = n*[1/2*dC4;-C1];  
        u = [kA*u_A;-kB*n*(C5-C5d)];
        r = state(i,1:3).'*2; % 2衛星を考慮して2倍にする
        [myu1, myu2] = ru2myu(r,u, coilN, radius, I_max);
        %u = [0;0;0];
        if norm(myu1) > myu_max
            u = myu_max/2* u/norm(myu1);
            myu1 = myu_max/2 * myu1/norm(myu1);
            %disp("over myu1")
        end
        
    else 
        k_avoid = 1e-1;
        u = k_avoid * u_max * X(1:3)/norm(X(1:3));
        disp("avoid")
    end
    %u = [0;0;0];
    myu_list(3*N-3*(i-1)-2:3*N-3*(i-1)) = myu1;
    state(i+1,:) = (A_d * state(i,:).' + B_d * u).';
    s(6*N*2 - 6*2*(i-1)-11:6*N*2 - 6*2*(i-1)-6) = (A_d * state(i,:).' + B_d * u);
    s(6*N*2 - 6*2*(i-1)-5:6*N*2 - 6*2*(i-1)) = -(A_d * state(i,:).' + B_d * u);

end

satellites{1} = state(:,1:3);

u_myu =  myu_list;
%plot_s(satellites, num, N, rr, d_target)
disp("各エネルギー")
x_r = state(N+1,:).';
kA = 2e-3; % 2e-3
thetaP = pi/6;
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];
error = mat * x_r;
disp(error)
disp("エネルギー総和")
disp(sum(abs(error)))


%% 最適化2

clearvars myu_mat
clearvars s_mat
tic;
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
disp("最大電力設定")
disp(P_max)
disp("最大電流設定")
disp(I_max)
disp("最大磁気モーメント設定")
disp(myu_max)

d_avoid = radius*6;
% 初期衛星間距離
d_initial = d_avoid/2;
s01 = [d_initial; d_initial; -0.00005; 0; 0; 0];
s02 = [-d_initial; -d_initial; 0.00005; 0; 0; 0];
s0 = adjust_cog([s01, s02], num); % 6num×1

%dt=10の10ステップぐらいの最適化だったらこれくらいのtrust regionでいい。
%delta_r = d_avoid/10;
%delta_myu = myu_max/10;

delta_r = d_avoid/10;
delta_myu = myu_max/10;

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

A_ = A;
B_ = B;

for i = 2:num
    A_ = blkdiag(A_, A); % BにAを対角に追加
    B_ = [B_; B]; % BにAを対角に追加
end

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num
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
disp("線形化したEMFFダイナミクスを用いて、線形誤差を計算。小さかったら問題なし")
l_s = P * u_myu + Q * s0 + R;
disp(l_s(1:6) - s(1:6))

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
A = [A2;A3;A4];
b = [b2;b3;b4];
 

kA = 2e-3;
thetaP = pi/6;
relative_mat = [eye(6),-eye(6)];
mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
       2,0,0,0,1/n,0;
       0,0,0,-1/(n*tan(thetaP)),0,1/n;
       -1/(n*tan(thetaP)),0,1/n, 0,0,0];

[x, fval, exitflag, output] = solveOptimizationProblem(n, u_myu, N, myu_max, P, Q, R, s0, d_avoid, A, b);


% 衛星の状態
s = P * x + Q * s0 + R;
s1 = s;
u_myu = x;
myu_mat = reshape(x, 3, N).'; 
disp("最大磁気モーメント myu_max")
disp(max(vecnorm(myu_mat,2,2)))

disp("最大電力")
disp(R_rho*(max(vecnorm(myu_mat,2,2))/(coilN * radius^2 * pi))^2)

disp("安定チェック")
error = mat * s(1:6);
disp(error)

disp("最小化したい評価値")
disp(sum(abs(error)))

time = toc
%% 図示

plot_s(s, num, N, rr, d_target)

%% 関数リスト1
function [x, fval, exitflag, output] = solveOptimizationProblem(n, x0, N, myu_max, P, Q, R, s0, d_avoid, A, b)
    % 初期化
    %A = [];  % 不等式制約 A*x <= b の A
    %b = [];  % 不等式制約 A*x <= b の b
    Aeq = [];
    beq = []; 
    lb = []; % 変数の下限
    ub = []; % 変数の上限

    % オプションの設定
    options = optimoptions('fmincon', ...
                       'Algorithm', 'sqp', ...
                       'TolFun', 1e-6, ...
                       'TolX', 1e-6, ...
                       'TolCon', 1e-6, ...
                       'Display', 'iter', ...
                       'MaxIterations', 400, ...
                       'StepTolerance', 1e-6);


    fun =  @(x) objectiveFunction(n, x, P, Q, R, s0);

    c_ceq = @(x) nonlinearConstraints(x, N, myu_max, P, Q, s0, d_avoid);

    % fminconの呼び出し
    [x, fval, exitflag, output] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, c_ceq, options);
end

function f = objectiveFunction(n, x, P, Q, R, s0)
    % 目的関数の計算
    kA = 2e-3;
    thetaP = pi/6;
    %relative_mat = [eye(6),-eye(6)];
    mat = [-6*n/kA,1,0,-2/n,-3/kA,0;
           2,0,0,0,1/n,0;
           0,0,0,-1/(n*tan(thetaP)),0,1/n;
           -1/(n*tan(thetaP)),0,1/n, 0,0,0];
    
    f = sum(abs(mat * (P(1:6,:) * x + Q(1:6,:) * s0 + R(1:6,:)))); % xを用いた計算
end

function [c, ceq] = nonlinearConstraints(x, N, myu_max, P, Q, s0, d_avoid)
    
    % 非線形制約の計算
    % 発電量拘束
    
    P_list = zeros(N,1);
    for i = 1:N
        P_list(i) = norm(x(3*(i-1)+1:3*i)) - myu_max;   
    end
    

    % 進入禁止制約
    dist_margin_list = zeros(N,1);
    relative_mat = [eye(6),-eye(6)];
    s = P * x + Q * s0;
    for i = 1:N
        dist_margin_list(i) = d_avoid/2 - norm(s(6*(i-1)+1:6*(i-1)+3));   
    end

    % 制御可能範囲制約
    %%%%%%%%%%%%%%%%%%%
    
    c = [P_list; dist_margin_list];  % 不等式制約 c(x) <= 0 進入禁止制約、発電量拘束90-iop[k
    ceq = [];% 等式制約 ceq(x) = 0
end


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
    for i = 1:1:N
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



%% 関数リスト2
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



