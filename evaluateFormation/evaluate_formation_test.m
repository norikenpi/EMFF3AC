% 詳細は0925の発表資料を見てみる
% 制御する偏差を表現するD z=D^T x
% 衛星間の接続を表現するD_　u = [D_ D_][e e_dot]^T


%一定の領域内の誤差を仮定
delta = 0.01;
delta_dot = 0.001;

%定数
myu0 = param.myu0;
d = param.satellite_desired_distance;
N = param.coilN;
R = param.radius;
m = param.mass;
C = 3*myu0*N^2*pi*R^4/(4*m);
Imax = param.I_max;

%ゲイン

D = [-1, 0;
     1, -1;
     0, 1];

D = kron(D, eye(3));
k0_pos = C * Imax^2/(d^4 * delta);
k0_vel = C * Imax^2/(d^4 * delta_dot);

%フォーメーションのネットワークを定義
% 三角
D_1 = [-2, -1;
     1, -1;
     1, 2];
k_1pos = k0_pos/4;
k_1vel = k0_vel/4;
D1_ = kron(D_1, eye(3));

% 棒
D_2 = [-1, 0;
     1, -1;
      0, 1];
k_2pos = k0_pos/2;
k_2vel = k0_vel/2;
D2_ = kron(D_2, eye(3));

Acl1 = [zeros(6), eye(6); -D.' * k_1pos * D1_, -D.' * k_1vel * D1_];
Acl2 = [zeros(6), eye(6); -D.' * k_2pos * D2_, -D.' * k_2vel * D2_]; 

eigenvalues1 = eig(Acl1); % eig() 関数で固有値を計算
eigenvalues2 = eig(Acl2); % eig() 関数で固有値を計算

disp('固有値1:');
format long
disp(eigenvalues1);
disp(max(real(eigenvalues1)));

disp('固有値2:');
format long
disp(eigenvalues2);
disp(max(real(eigenvalues2)));