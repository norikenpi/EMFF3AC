%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。

assignin('base', 'current', histories.magnetic_moment_histories/(param.coilN*pi*param.radius^2));


numSatellites = size(current, 3);  % 衛星の数
numTimePoints = size(current, 1);  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}
time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
figure
current_data = zeros(param.N, numTimePoints);
for i = 1
    current_data(i, :) = vecnorm(current(:,:,i).');  % 各衛星の位置データを抽出し、行列に格納
    plot(time, current_data(i,:))
    disp(i)
end

plot(time, current_data)
xlabel('Time(sec)')
ylabel('current(A)')
title('current vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1')  % 必要に応じて衛星の数に応じて拡張

%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
savePlot(param, fileName)

