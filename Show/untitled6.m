%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。
%untitled3,4,5はシミュレーションたいむステップを変えたときにどれくらい誤差が出るかをチェックするもの。
satellite_i = 7;
satellite_j = 6;

assignin('base', 'u', histories.u_histories);
assignin('base', 'position', histories.position_histories);


t_final = 500;

numTimePoints = round(param.t/param.dt);  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}

r6 = position(:, :, 6);
r7 = position(:, :, 7);

r76 = r7 - r6;
u_r = zeros(numTimePoints, 1);
%位置ベクトル方向の力
for idx = 1:numTimePoints
    u_r(idx) = dot(r76(idx, :), u(idx, :, 7));
end

time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
%time = 0:param.dt:t_final;  % x軸の時間データ

colors = jet(size(histories.position_histories, 3)); % N個の衛星に対して異なる色を設定

figure
u_norm = zeros(param.N, numTimePoints);

%凡例を追加
labels = cell(0);
u_norm(satellite_i, :) = u_r;  % 各衛星の位置データを抽出し、行列に格納
plot(time(1,1:t_final/param.dt), u_norm(satellite_i,1:t_final/param.dt), 'Color', colors(satellite_i, :))
hold on
disp(satellite_i)
labels{end + 1} = sprintf('Satellite %s', string(satellite_i));

ylim([-12*10^-3 8*10^-3]);

xlabel('Time(sec)')
ylabel('相対位置ベクトル方向の所望の力')
title('衛星6から見た衛星7の位置ベクトル方向の衛星7に加えようとする力')

% 衛星ごとに凡例を追加
legend(labels)
%{
%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
filename_option = string(satellite_i);
fileName = sprintf("%s_%s", fileName, filename_option);

savePlot(param, fileName)
%}