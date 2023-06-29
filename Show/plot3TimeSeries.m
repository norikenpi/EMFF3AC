% 3次元座標の時系列データを図示
figure;

assignin('base', 'force12', -histories.force_histories(:,:,1));
assignin('base', 'x1', histories.position_histories(:,:,1));
assignin('base', 'x2', histories.position_histories(:,:,2));

assignin('base', 'v1', histories.velocity_histories(:,:,1));
assignin('base', 'v2', histories.velocity_histories(:,:,2));

x2x1 = x2 - x1; 
v2v1 = v2 - v1;

% 抽出する要素のインデックスを計算
startIndex = 2089; % 最初の要素のインデックス
stepSize = 21; % 抽出する要素の間隔
indices = startIndex:stepSize:length(force12(2070:end,:));

force1 = force12(indices, :);

data = force1(:, :)./vecnorm(force1(:, :), 2,2);
data = x2x1(2070:end,:);
% 色のグラデーションを作成
color_map = jet(size(data, 1)); % ジェットカラーマップを使用して100段階の色を作成
scatter(data(:, 1), data(:, 3), 50, color_map, 'filled');
hold on;
%h1 = scatter(0, 0, 50,'k', 'filled');
xlabel('X軸');
ylabel('Z軸');
title('衛星2から見た衛星1の軌道');
%xlim([-1, 1]);  % x軸の範囲を-1から1に設定
%ylim([-1, 1]);  % y軸の範囲を-1から1に設定
%legend(h1, 'origin');
grid on;
set(gca,'YDir','reverse')
daspect([1 1 1]);