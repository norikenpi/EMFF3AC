% ビデオライターオブジェクトの作成
v = VideoWriter('simple_harmonic_motion.avi'); % AVIファイル形式で動画を保存
open(v);

% フィギュアの作成
figure;
axis equal;
xlim([-1, 1]); % x軸の範囲を調整
ylim([-1, 1]); % y軸の範囲を調整
zlim([-1, 1]); % z軸の範囲を調整
hold on;
grid on; % グリッドを表示

% 点の初期位置と振動パラメータ
initial_position = [0, 0, 0];
amplitude = 0.5; % 振幅
frequency = 2; % 周波数
time = linspace(0, 4 * pi, 400); % 時間軸

% 各フレームでの点の位置をプロットし、動画に書き込む
for t = time
    % 軌跡をクリア（前のフレームの内容を消去）
    cla;
    
    % フィギュアの設定を再度適用
    axis equal;
    xlim([-1, 1]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    hold on;
    grid on;

    % 単振動の式から点の位置を計算
    x = initial_position(1) + amplitude * sin(frequency * t);
    y = initial_position(2) + amplitude * sin(frequency * t);
    z = initial_position(3) + amplitude * sin(frequency * t);

    % 点をプロット
    plot3(x, y, z, 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');

    % 視点を変更
    azimuth = 45; % 方位角
    elevation = 30; % 仰角
    view(azimuth, elevation);

    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% ビデオの保存
close(v);
