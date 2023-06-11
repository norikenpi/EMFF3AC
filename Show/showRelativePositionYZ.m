satellite_i = 6;
satellite_j = 7;
final = 1000/param.dt;
relative_position = histories.position_histories(1:final,:,satellite_j) - histories.position_histories(1:final,:,satellite_i);




% プロットの作成
figure;
plot(relative_position(:, 2), relative_position(:, 3), 'b');
xlabel('Y(m)(面外方向)');
ylabel('Z(地心方向)');
title('衛星の相対軌道　50秒おきに赤丸 6から見た7');

hold on
scatter(relative_position(1:50/param.dt:end, 2), relative_position(1:50/param.dt:end, 3), 'r');
scatter(relative_position(1, 2), relative_position(1, 3), 'k');

% グリッドの表示
grid on;
%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
filename_option = string(satellite_i);
fileName = sprintf("%s_%s", fileName, filename_option);

%z軸を逆向き
set(gca,'YDir','reverse')
xlim([0 0.005])

savePlot(param, fileName)