%% パラメータ設定

% 地球を周回する衛星の角速度
% n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度 宇宙ステーション入門 p107
n = 0.0011; % 0.0011

% 衛星質量
m = 1; % 1

% タイムステップ(s)
dt = 1;

% 時間 N×dt秒
N = 10000;

% 衛星数
num = 2;

% 進入禁止範囲(m)（進入禁止制約を設定しない場合は-1にしてください）
%R = 0.0;
R = -1;

% trust region
delta = 0.0;
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


% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6) + dt*A_; % 6num×6num
B_d = dt*B_; % 6num×3num

% 初期状態
s01 = [0.5; 0.1; 0.1; 0.1; 0.1; 0.1];
s02 = [-0.5; -0.0000001; -0.0000001; 0; 0; 0];
s0 = [s01; s02]; % 6num×1

% 2衛星のそれぞれの目標状態
sd1 = [-0.5; 0.5; 0; 0; 0; 0];
sd2 = [0.5; -0.5; 0; 0; 0; 0];
sd = [sd1; sd2];

%s1 = A_d * s0 + B_d * u
theta = 0;
r = 1;
k1 = 0.01;
k2 = 0.01;
k3 = 0.01;

s = s01;
s_data = zeros(6*N);
for i = 1:N
    x = s(1);
    y = s(2);
    z = s(3);
    vx = s(4);
    vy = s(5);
    vz = s(6);
    disp(x)
    C1 = -vx/n + 2*z;
    C4 = -x - 2*vz/n;
    C5 = vy/n;
    C6 = y;
    delta_z = C5^2 + C6^2 - r^2*cos(theta)^2;
    ux = k2*n*C4/2 - 3*n^2*C1/2;
    uy = -k1*n*C1;
    uz = -k3*delta_z*n/C5;
    u = [ux; uy; uz];
    s = A_d * s + B_d * u;
    s_data(6*(N-i)+1:6*(N+1-i)) = s;
end


%% 図示

% 2衛星の動画を表示。
data = reorderMatrix(s_data);

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