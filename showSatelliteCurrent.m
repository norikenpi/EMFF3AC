%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。

assignin('base', 'magnetic_moment', histories.magnetic_moment_histories/(param.radius^2*param.coilN*pi));
numSatellites = size(magnetic_moment, 3);  % 衛星の数
numTimePoints = size(magnetic_moment, 1);  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}
time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
figure
current_data = zeros(param.N, numTimePoints);
for i = 1:numSatellites
    
    current_data(i, :) = vecnorm(magnetic_moment(:,:,i).');  % 各衛星の位置データを抽出し、行列に格納
    plot(time, current_data(i,:))
    disp(i)
end



plot(time, current_data)
xlabel('Time')
ylabel('current')
title('Current vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1', 'Satellite 2', 'Satellite 3')  % 必要に応じて衛星の数に応じて拡張