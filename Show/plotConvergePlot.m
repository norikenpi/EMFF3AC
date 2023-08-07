%収束時間を図示
%この関数を実行するとparamに設定されたフォルダにfigとjpegが保存される．
function plotConvergePlot(array, first_seed, type, param)
    figure
    % インデックスを生成
    index = first_seed:(size(array, 2) + first_seed - 1);
    hold on
    for i = 1:length(array)
        if array(i) < 0
            plot(index(i), array(i), 'ro'); % マイナスの値を赤色でプロット
        else
            plot(index(i), array(i), 'bo'); % プラスの値をデフォルトの色でプロット
        end
    end
    % プロット
    %plot(index, array, 'o');
    xlabel('Seed');
    ylim([-2000, 2000])
    ylabel('Converge time(s)');
    title(sprintf('Plot of Converge time(2cm) %s', type));
    %グラフ画像に日付を入れる。
    xPosition = xlim; % x軸の範囲を取得
    yPosition = ylim*0.8; % y軸の範囲を取得
    %text(xPosition(2), yPosition(2), string(param.date), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

    grid on
    hold off

    mkdir(param.path_data)
    filename_fig = strcat(param.path_data, sprintf('/%s.fig', type));
    filename_jpg = strcat(param.path_data, sprintf('/%s.jpg', type));

    saveas(gcf, filename_fig);
    saveas(gcf, filename_jpg);