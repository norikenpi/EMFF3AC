%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。
%untitled3,4,5はシミュレーションたいむステップを変えたときにどれくらい誤差が出るかをチェックするもの。

satellite_i = 1;

assignin('base', 'position001', histories.position_histories);


t_final = 500;

numTimePoints = round(param.t/param.dt)+1;  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}


time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
%time = 0:param.dt:t_final;  % x軸の時間データ

colors = jet(size(histories.position_histories, 3)); % N個の衛星に対して異なる色を設定

figure
position_norm = zeros(param.N, numTimePoints);

%凡例を追加
labels = cell(0);
position_norm(satellite_i, :) = vecnorm(position001(:,:,satellite_i).');  % 各衛星の位置データを抽出し、行列に格納
plot(time(1,1:t_final/param.dt), position_norm(satellite_i,1:t_final/param.dt), 'Color', colors(satellite_i, :))
hold on
disp(satellite_i)
labels{end + 1} = sprintf('Satellite %s', string(satellite_i));

%ylim([0 5000]);

xlabel('Time(sec)')
ylabel('current-req(A)')
title('current-req vs. Time')

% 衛星ごとに凡例を追加
legend(labels)
%{
%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
filename_option = string(satellite_i);
fileName = sprintf("%s_%s", fileName, filename_option);

savePlot(param, fileName)
%}