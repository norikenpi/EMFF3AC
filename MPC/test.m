%台形近似の係数
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
sd = repmat([0.5; 0.5; 0; 0; 0; 0], N, 1);

%S = PU + Qs_0
P = controllability_matrix(A_d, B_d, N); %6n×3n
Q = controllability_matrix2(A_d, N); %6n×6

%最大加速度
u_max = ones(3*N, 1)*10^(-5);

%評価関数(位置に関する関数)
%J = xKx
K = zeros(6*N); % 6n×6n
K(1:3, 1:3) = eye(3); % 最終位置のみ考慮


%評価関数(時系列入力に関する関数)
%J = uKu
H = P.' * K * P;% 3n×3n
f = (s0.' * Q.' - sd.')* K * P; % 1×3n

%拘束条件(時系列入力に関する関数)
% u_min u < u_max
A = eye(3*N);
ub = u_max;
lb = -u_max;



% 入力を最適化
[x,fval,exitflag,output,lambda] = ...
   quadprog(H, f, [], [], [], [], lb, ub);

% 求まった入力
s = P * x + Q * s0;
