% 定義域を指定
t = linspace(0, 100, 1000);  % 0から5までの範囲で1000ポイントを生成

% 関数を計算
y1 = exp(-0.0545 * t);
y2 = exp(-0.1091 * t);

% グラフをプロット
figure;             % 新しいフィギュアウィンドウを開く
hold on;            % 複数のグラフを同時に表示するための設定

plot(t, y1, 'r', 'LineWidth', 2);  % e^tを赤色でプロット
plot(t, y2, 'b', 'LineWidth', 2);  % e^{2t}を青色でプロット

title('Graphs of -e^{-0.0545t} and e^{-0.1091t}'); % タイトル
xlabel('t(sec)');        % x軸のラベル
ylabel('Error(cm)');    % y軸のラベル
legend('-e^{-0.0545t}', 'e^{-0.1091t}'); % 凡例
grid on;            % グリッドをオンにする
legend('x2 - x1', 'Data 2', 'Data 3'); % 凡例

hold off;           % ホールドオフ
