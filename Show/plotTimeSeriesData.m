function plotTimeSeriesData(data)
    % プロットして図示する
    hold off
    plot(data);

    % グラフの装飾
    title('データの時間経過');
    xlabel('時間');
    ylim([0.018, 0.027]);
    xlim([0, 80.0]);
    ylabel('値');
    grid on; % グリッドを表示
end