

s = s1;
data = reorderMatrix(s);

% ビデオライターオブジェクトの作成
v = VideoWriter('points_motion.avi'); % AVIファイル形式で動画を保存
open(v);

% フィギュアの作成
figure;
axis equal;
xlim([-1, 1]); % xの範囲を調整
ylim([-1, 1]); % yの範囲を調整
hold on;

% 各フレームでの点の位置をプロットし、そのフレームを動画に書き込む
for i = 1:10:length(data)-1
    disp(i)
    plot(data(i), data(i+1), 'o', 'MarkerSize', 10);
    drawnow;
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% ビデオの保存
close(v);