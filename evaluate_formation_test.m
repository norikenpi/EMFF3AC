% 詳細は0925の発表資料を見てみる
% 制御する偏差を表現するD z=D^T x
% 衛星間の接続を表現するD_　u = [D_ D_][e e_dot]^T

%{
A1 = [1/2, -1/2, 0;
     -1/2, 1, -1/2;
     0, -1/2, 1/2]; % 固有値を計算したい行列を定義

A2 = [1/2, -1/4, -1/4;
     -1/4, 1/2, -1/4;
     -1/4, -1/4, 1/2]; % 固有値を計算したい行列を定義

A3 = [1/2, -1/4, -1/4;
     -1/4, 1/2, -1/4;
     -1/4, -1/4, 1/2]/2^4; % 固有値を計算したい行列を定義
%}

%衛星間距離
d = 1;

%一定の領域内の誤差を仮定
delta = 0.01;
delta_dot = 0.01;

%定数
myu0 = 4*pi*1e-7;
N = 800;
R = 0.05;
C = 3*myu0*N*R^2/4;


%最大電流
Imax = 1;

%ゲイン
k = C * Imax^2/(d^4 * delta);

D = [-1, 0;
     1, -1;
     0, 1];

D = kron(D, eye(3));

%フォーメーションのネットワークを定義
% 三角
D_1 = [-2, -1;
     1, -1;
     1, 2] * k/4;

% 棒
D_2 = [-1, 0;
     1, -1;
      0, 1]*k/2;

D_ = D_1;

D_ = kron(D_, eye(3));

kD_ = D_;
kLe = D.' * kD_;

Acl = [zeros(6), eye(6); -kLe, -kLe];

eigenvalues51 = eig(Acl); % eig() 関数で固有値を計算

D_ = D_2;

D_ = kron(D_, eye(3));

kD_ = D_;
kLe = D.' * kD_;

Acl = [zeros(6), eye(6); -kLe, -kLe];

eigenvalues52 = eig(Acl); % eig() 関数で固有値を計算

disp('固有値1:');
format long
disp(max(eigenvalues51));

disp('固有値2:');
format long
disp(max(eigenvalues52));


%disp(eigenvectors5)


%[eigenvectors1, eigenvalues1] = eig(A1); % eig() 関数で固有値を計算
%[eigenvectors2, eigenvalues2] = eig(A2); % eig() 関数で固有値を計算
%[eigenvectors3, eigenvalues3] = eig(A3); % eig() 関数で固有値を計算
%[eigenvectors4, eigenvalues4] = eig(-kLe); % eig() 関数で固有値を計算

% 結果を表示
%disp('固有値1:');
%disp(eigenvalues1);
%disp(eigenvectors1)
%disp('固有値2:');
%disp(eigenvalues2);
%disp(eigenvectors2)
%disp('固有値3:');
%disp(eigenvalues3);
%disp(eigenvectors3)
%disp('固有値4:');
%disp(eigenvalues4);
%disp(eigenvectors4)
