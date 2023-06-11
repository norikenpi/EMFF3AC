%発生させようとした力と実際に発生した力を比較
%電流の上限にどこで引っかかったかをチェック。

assignin('base', 'current_req', histories.magnetic_moment_req_histories/(param.coilN*pi*param.radius^2));


t_final = 200;

numTimePoints = round(param.t)/param.dt;  % 時間データの数

%{
error_data = zeros(numTimePoints, numSatellites);  % データを格納する行列

for satellite_i = 1:numSatellites
    error_data(:, satellite_i) = vecnorm(u{satellite_i} - u_real{satellite_i});  % 各衛星の位置データを抽出し、行列に格納
end
%}


time = 0:param.dt:(numTimePoints-1)*param.dt;  % x軸の時間データ
%time = 0:param.dt:t_final;  % x軸の時間データ


figure
current_req_data = zeros(param.N, numTimePoints);
for i = 2
    current_req_data(i, :) = vecnorm(current_req(:,:,i).');  % 各衛星の位置データを抽出し、行列に格納
    disp(i)
end

plot(time(1,1:t_final/param.dt), current_req_data(:,1:t_final/param.dt))
xlabel('Time(sec)')
ylabel('current-req(A)')
title('current-req vs. Time')

% 衛星ごとに凡例を追加
legend('Satellite 1')  % 必要に応じて衛星の数に応じて拡張

%画像を保存
[~, fileName, ~] = fileparts(mfilename('fullpath'));
savePlot(param, fileName)
