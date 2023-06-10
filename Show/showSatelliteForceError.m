%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。

assignin('base', 'u', histories.u_histories);
assignin('base', 'u_real', histories.u_real_histories);
assignin('base', 'force', histories.force_histories);

numSatellites = size(u, 3);  % 衛星の数
numTimePoints = size(u, 1);  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}
time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
figure
error_data = zeros(param.N, numTimePoints);
for i = 1:param.N
    
    error_data(i, :) = vecnorm(u(:,:,i).' - u_real(:,:,i).');  % 各衛星の位置データを抽出し、行列に格納
    plot(time, error_data(i,:))
    disp(i)
end



plot(time, error_data)
xlabel('Time')
ylabel('error')
title('error vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1', 'Satellite 2', 'Satellite 3')  % 必要に応じて衛星の数に応じて拡張

[~, fileName, ~] = fileparts(mfilename('fullpath'));
savePlot(param, fileName)