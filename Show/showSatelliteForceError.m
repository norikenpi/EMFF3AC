%各衛星のC1の時間変化のグラフ

assignin('base', 'u', histories.u_histories);
assignin('base', 'u_real', histories.u_real_histories);
assignin('base', 'force', histories.force_histories);

numSatellites = length(u);  % 衛星の数
numTimePoints = size(u{1}, 2);  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}

error_data = zeros(numTimePoints, 1);  % データを格納する行列
error_data(:, 1) = vecnorm(u{1} - u_real{1});  % 各衛星の位置データを抽出し、行列に格納


time = 1:numTimePoints;  % x軸の時間データ
figure
plot(time, error_data)
xlabel('Time')
ylabel('error')
title('error vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1', 'Satellite 2', 'Satellite 3')  % 必要に応じて衛星の数に応じて拡張