%untitled3,4,5はシミュレーションたいむステップを変えたときにどれくらい誤差が出るかをチェックするもの。

%6/16のスライドにのってる。
error_data005 = position005_56770 - position001_56770;
error_data01 = position01_56770 - position001_56770;

error_data005_norm = vecnorm(error_data005, 2, 2);
error_data01_norm = vecnorm(error_data01, 2, 2);


% 時間軸の作成
time = (0.1:0.1:5677)';  % 0.1秒刻みで10秒間

% プロット
figure;
plot(time, error_data005_norm, 'b', 'LineWidth', 2);
hold on;
plot(time, error_data01_norm, 'r', 'LineWidth', 2);

% グラフの設定
title('シミュレーションタイムステップによる位置の違い（Δt=0.01sとの比較）');
xlabel('時間（秒）');
ylabel('位置のずれ(m)');
legend('Δt=0.05s', 'Δt=0.1s');
grid on;