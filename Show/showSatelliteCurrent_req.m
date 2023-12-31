%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。

satellite_i = 1;

assignin('base', 'current_req', histories.magnetic_moment_req_histories/(param.coilN*pi*param.radius^2));


t_final = 500;

numTimePoints = round(param.t)/param.dt;  % 時間データの数

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
current_req_data = zeros(param.N, numTimePoints);

%凡例を追加
labels = cell(0);
current_req_data(satellite_i, :) = vecnorm(current_req(:,:,satellite_i).');  % 各衛星の位置データを抽出し、行列に格納
plot(time(1,1:t_final/param.dt), current_req_data(satellite_i,1:t_final/param.dt), 'Color', colors(satellite_i, :))
hold on
disp(satellite_i)
labels{end + 1} = sprintf('Satellite %s', string(satellite_i));

ylim([0 5000]);

xlabel('Time(sec)')
ylabel('current-req(A)')
title('current-req vs. Time')

% 衛星ごとに凡例を追加
legend(labels)

%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
filename_option = string(satellite_i);
fileName = sprintf("%s_%s", fileName, filename_option);

savePlot(param, fileName)
