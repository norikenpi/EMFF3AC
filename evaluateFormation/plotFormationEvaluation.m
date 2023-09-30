%histories.relative_distance_pair
A1 = histories.relative_distance_pair(:,1);
A2 = histories.relative_distance_pair(:,2);
A3 = histories.relative_distance_pair(:,3);
%disp(A)

% グラフをプロット
figure;                 % 新しいフィギュアウィンドウを開く
hold on;
plot(linspace(0,param.t,length(A1)), A1, '-o'); % 列ベクトルの値を縦軸にとったグラフをプロット
plot(linspace(0,param.t,length(A2)), A2, '-o');
plot(linspace(0,param.t,length(A3)), A3, '-o');
xlabel('時間（sec）');        % x軸のラベル
ylabel('平均位置誤差（cm）');    % y軸のラベル
title('フォーメーション1'); % タイトル
grid on;                % グリッドをオンにする
legend('z2 - z1', 'z3 - z2', 'z1 - z3'); % 凡例