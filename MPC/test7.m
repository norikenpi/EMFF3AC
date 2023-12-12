figure;
x = 0.18:0.1:3; % xの範囲を-10から10まで1の間隔で定義
y = 1 ./ (x.^4) * 0.18^4 * 100;         % y = x の関係を使用
plot(x, y);    % グラフを描画
grid on;       % グリッドを表示
xlabel('x');   % x軸のラベル
ylabel('y');   % y軸のラベル
xlim([0.18, 2]);
title('y = 1/x^4 のグラフ(0.18m地点を100とした)'); % グラフのタイトル