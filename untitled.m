% パラメータの設定
x = linspace(0, 2*pi * 3, 100); % x軸の範囲を0から2πまで100等分した点の配列を生成

% 三角関数の計算
y1 = sin(60*x).*sin(40*x); % sin関数を計算し、結果をy1に格納
y2 = sin(x).*sin(x); % cos関数を計算し、結果をy2に格納
y3 = sin(x).*sin(2*x).*sin(3*x);

% プロットの作成
figure; % 新しい図を作成
hold on; % 複数のグラフを同時に表示するためにhold onを使用
plot(x, y1, 'r-', 'LineWidth', 2); % xとy1をプロット（赤色で線幅2）
plot(x, y2, 'b--', 'LineWidth', 2); % xとy2をプロット（青色の破線で線幅2）
plot(x, y3, 'g--', 'LineWidth', 2); % xとy2をプロット（青色の破線で線幅2）
hold off; % グラフの描画を終了

% グラフの装飾
title('三角関数のプロット'); % グラフのタイトル
xlabel('x'); % x軸のラベル
ylabel('y'); % y軸のラベル
legend('sin(x)×sin(2x)', 'sin(x)'); % 凡例を表示（sin(x)とcos(x)）
grid on; % グリッドを表示

% グラフの表示
