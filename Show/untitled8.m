% 分離する衛星の相対位置とか相対速度とかを図示する。
figure;

assignin('base', 'force12', -histories.force_histories(:,:,1));
assignin('base', 'x1', histories.position_histories(:,:,1));
assignin('base', 'x2', histories.position_histories(:,:,2));

assignin('base', 'v1', histories.velocity_histories(:,:,1));
assignin('base', 'v2', histories.velocity_histories(:,:,2));

assignin('base', 'x12', histories.position_histories(:,:,12));
assignin('base', 'x16', histories.position_histories(:,:,16));

assignin('base', 'v12', histories.velocity_histories(:,:,12));
assignin('base', 'v16', histories.velocity_histories(:,:,16));

x2x1 = x2 - x1; 

x16x12 = x16 - x12;
v16v12 = v16 - v12;

% 抽出する要素のインデックスを計算
startIndex = 2089; % 最初の要素のインデックス
stepSize = 21; % 抽出する要素の間隔
indices = startIndex:stepSize:length(force12(2070:end,:));

force1 = force12(indices, :);

data = force1(:, :)./vecnorm(force1(:, :), 2,2);
data = v16v12(:,2);
% 色のグラデーションを作成
color_map = jet(size(data, 1)); % ジェットカラーマップを使用して100段階の色を作成

time = 0:0.1:(length(data)-1)/10;

plot(time, data);
hold on;
%h1 = scatter(0, 0, 50,'k', 'filled');
xlabel('時間（秒）');
ylabel('y軸');
title('衛星12から見た衛星16のy軸方向の相対位置');
%xlim([-1, 1]);  % x軸の範囲を-1から1に設定
%ylim([-1, 1]);  % y軸の範囲を-1から1に設定
%legend(h1, 'origin');
grid on;
%set(gca,'YDir','reverse')
%daspect([1 1 1]);