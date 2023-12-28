clear
rng(1)

num = 5;
grid_num = 3;

% 点の間隔を設定
initial_d = 10; % 例として10を使用

% 3x3のグリッドの座標を生成
[X, Y] = meshgrid(0:initial_d:2*initial_d, 0:initial_d:2*initial_d);
Z = zeros(grid_num,grid_num);
pos_rand = (2*rand(size([X(:) Y(:), Z(:)]))-1)*1e-3;
vel = (2*rand(size([X(:) Y(:), Z(:)]))-1)*1e-5;
% 7点分の座標を選択
pos = [X(:) Y(:), Z(:)] + pos_rand;
s0 = [pos(1:num, :, :), vel(1:num, :, :)].'