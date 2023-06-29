% パラメータの設定
x = linspace(0, 1, 1000); % x軸の範囲を0から2πまで100等分した点の配列を生成

% 三角関数の計算
y1 = sin(2*pi*x); % sin関数を計算し、結果をy1に格納
y2 = sin(4*pi*x); % cos関数を計算し、結果をy2に格納
y3 = sin(2*pi*x).*sin(2*pi*x);

% プロットの作成
figure; % 新しい図を作成
hold on; % 複数のグラフを同時に表示するためにhold onを使用
%plot(x, y1, 'r-', 'LineWidth', 1); % xとy1をプロット（赤色で線幅2）
%plot(x, y2, 'b-', 'LineWidth', 1); % xとy2をプロット（青色の破線で線幅2）
plot(x, y3, 'k-', 'LineWidth', 1); % xとy2をプロット（青色の破線で線幅2）
ylim([-1 1])
hold off; % グラフの描画を終了

% グラフの装飾
title('三角関数のプロット'); % グラフのタイトル
xlabel('t'); % x軸のラベル
ylabel('y'); % y軸のラベル
legend('sin(2πt)×sin(2πt)'); % 凡例を表示（sin(x)とcos(x)）

grid on; % グリッドを表示

% グラフの表示
