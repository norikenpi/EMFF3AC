%衛星2つに関して最適化を行う。
%力の和が0という線形制約を追加することで運動量保存を実現。

%決められた時間で、入力の最大値が最小化するような　プロファイルを計算。
%入力プロファイルの一番最後に最大入力変数を格納
% quadprogでやったら非凸って言われたから解けてない。
% 初期値を設定する際に入力上限を0にしたほうがいいかもしれない。
%最終状態拘束

n = param.n;
m = param.mass;
dt = param.dt;
dt = 10;

%衛星数
num = 2;

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

%A_ = kron(A_,eye(num)); % 6num×6num
%B_ = kron(B_,eye(num)); % 6num×3num
A_ = [A_, zeros(6);zeros(6),A_];
B_ = [B_,zeros(6,3);zeros(6,3),B_];


A_d = eye(6*num) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

%最適化する時間。N×dt秒
N = 500;

%初期状態
s01 = [0.5; 0; 0; 0; 0; 0];
s02 = [-0.5; 0; 0; 0; 0; 0];

s0 = [s01; s02]; % 6num×1
s0 = reshape(permute(s0, [2 1]), [], 1); % 6Nnum×1

%目標状態
sd1 = [0; 0.3; 0; 0; 0; 0];
sd2 = [0; -0.3; 0; 0; 0; 0];
sd = [sd1; sd2];
%sd = [0;0;0.5;-0.5;0;0;0;0;0;0;0;0];
%sd = reshape(permute(sd, [2 1]), [], 1); % 6Nnum×1
%sd = repmat(sd, N, 1); % 6Nnum×1

%S = PU + Qs_0
P = controllability_matrix(A_d, B_d, N); %6Nnum×3Nnum
P = [P, zeros(6*N*num, 1)]; %6N×3Nnum+1
Q = controllability_matrix2(A_d, N); %6N×6num

%最大加速度
u_max = ones(3*N*num, 1)*10^(-5);

%評価関数1(位置に関する関数)
%J = xKx
K = zeros(6*num*N); % 6Nnum×6Nnum
K(1:3, 1:3) = eye(3); % 最終位置のみ考慮
K(7:9, 7:9) = eye(3);
%K = eye(6*num*N);


%評価関数1(時系列入力に関する関数)
%J = uHu + fU(3N+1)
H1 = P.' * K * P;% 3Nnum+1×3Nnum+1

%f1 = (s0.' * Q.' - sd.')* K * P; % 1×3Nnum+1

%評価関数2(最大入力最小)
%H2 = zeros(3*N*num + 1);% 3Nnum+1×3Nnum+1
H = zeros(6*N+1);
f2 = [zeros(1, 3*N*num), 1]; 
%lamda = 0.1;


H = H1;
%f = f1 + lamda * f2;
f = f2;

%拘束条件1(時系列入力に関する関数)
% 最大入力との差が0より大きくなければならない。
A = [eye(3*N*num), -ones(3*N*num,1); -eye(3*N*num), -ones(3*N*num, 1)]; %6N×3Nnum+1
b = zeros(6*N*num, 1);%6Nnum×1

%拘束条件2(時系列入力に関する関数 運動量保存)
Aeq1 = [ones(N, 3*N*num), zeros(N, 1)];
beq1 = zeros(N, 1);

%拘束条件3(最終状態固定)
Aeq2 = P(1:6*num,:);
beq2 = sd - Q(1:6*num,:) * s0; 

Aeq = [Aeq1; Aeq2];
beq = [beq1; beq2];



% 入力を最適化　　
[x,fval,exitflag,output,lambda] = ...
   linprog(f, A, b, Aeq, beq);


% 初期値
%{
x0 = zeros(3*N*num+1, 1);
x0 = x
%x0(3*N*num+1) = 0;
obj = @(x)0.5*x.'*H*x + f*x;

[x,fval,exitflag,output] = fmincon(obj, x0, A, b);
%}
% 求まった入力
s1 = P * x + Q * s0;

disp('Objective function value:');
disp(fval);
disp("u_max")
disp(x(3*num*N+1))