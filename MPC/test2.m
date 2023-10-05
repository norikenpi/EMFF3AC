%決められた時間で、入力の最大値が最小化するような　プロファイルを計算。
%入力プロファイルの一番最後に最大入力変数を格納
% quadprogでやったら非凸って言われたから解けてない。
% lamda=0にしてquadprogで解を求めて、それをfminconの初期値にする。
% 初期値を設定する際に入力上限を0にしたほうがいいかもしれない。

n = param.n;
m = param.mass;
dt = param.dt;
dt = 10;

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
     0, 0, 0, 0, 0, 0]/2;

B_ = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m];

A_d = eye(6) + dt*A_;
B_d = dt*B_;

%最適化する時間。N×dt秒
N = 500;

%初期状態
s0 = [0.1; 0.9; 0; 0; 0; 0];

%目標状態
sd = repmat([-0.5; -0.3; 0; 0; 0; 0], N, 1);

%S = PU + Qs_0
P = controllability_matrix(A_d, B_d, N); %6n×3n
P = [P, zeros(6*N, 1)]; %6n×3n+1
Q = controllability_matrix2(A_d, N); %6n×6

%最大加速度
u_max = ones(3*N, 1)*10^(-5);

%評価関数1(位置に関する関数)
%J = xKx
K = zeros(6*N); % 6n×6n
K(1:3, 1:3) = eye(3); % 最終位置のみ考慮


%評価関数1(時系列入力に関する関数)
%J = uKu + U(3N+1)
H1 = P.' * K * P;% 3n+1×3n+1
f1 = (s0.' * Q.' - sd.')* K * P; % 1×3n+1

%評価関数2(最大入力最小)
H2 = zeros(3*N + 1);
f2 = [zeros(1, 3*N), 1]; 
lamda = 0;

H = H1;
f = f1 + lamda * f2;

%拘束条件(時系列入力に関する関数)
% 最大入力との差が0より大きくなければならない。
A = [eye(3*N), -ones(3*N,1); -eye(3*N), -ones(3*N, 1)];
b = zeros(6*N, 1);


% 入力を最適化
[x,fval,exitflag,output,lambda] = ...
   quadprog(H1, f1, A, b);

% 初期値
%{
x0 = x;
x0(3*N+1) = 0;
obj = @(x)0.5*x.'*H*x + f*x;

[x,fval,exitflag,output] = fmincon(obj, x0, A, b);
%}
% 求まった入力
s = P * x + Q * s0;

disp('Objective function value:');
disp(fval);
disp("u_max")
disp(x(3*N+1))
