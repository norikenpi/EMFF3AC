% 1x100のランダムな配列を生成
function plotConvergePlot(array, first_seed, type)
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
    ylabel('Converge time(s)');
    title(sprintf('Plot of Converge time(2cm) %s', type));
    grid on
    hold off