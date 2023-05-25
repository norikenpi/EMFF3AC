%各衛星のC1の時間変化のグラフ
assignin('base', 'magnetic_moment_histories', histories.magnetic_moment_histories);
numSatellites = length(magnetic_moment_histories);  % 衛星の数
numTimePoints = size(magnetic_moment_histories{1}, 2);  % 時間データの数

moment_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    moment_data(:, satellite_i) = vecnorm(magnetic_moment_histories{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end

time = 1:numTimePoints;  % x軸の時間データ
figure
plot(time, moment_data)
xlabel('Time')
ylabel('magnetic moment')
title('magnetic moment vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1', 'Satellite 2', 'Satellite 3')  % 必要に応じて衛星の数に応じて拡張