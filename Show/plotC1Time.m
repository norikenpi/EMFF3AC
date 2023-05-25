%各衛星のC1の変化のグラフ
assignin('base', 'C1', histories.C1_histories);
numSatellites = length(C1);  % 衛星の数
numTimePoints = size(C1{1}, 2);  % 時間データの数

positionData = zeros(numTimePoints, numSatellites);  % データを格納する行列

for i = 1:numSatellites
    positionData(:, i) = C1{i}(1, :);  % 各衛星の位置データを抽出し、行列に格納
end

time = 1:numTimePoints;  % x軸の時間データ
figure
plot(time, positionData)
xlabel('Time')
ylabel('C1')
title('C1 vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1', 'Satellite 2', 'Satellite 3')  % 必要に応じて衛星の数に応じて拡張
