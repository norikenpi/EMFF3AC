function showBarplot(data1, data2, data3, data4, labels)
    figure;

    % 各配列の大きさ（要素数）を取得します
    dataSizes = [length(data1), length(data2), length(data3), length(data4)];
    
    % 棒グラフを作成します
    bar(dataSizes);
    % グラフのタイトルと軸ラベルを付けます
    title('速度の標準偏差と展開可否の関係')
    xlabel('速度の標準偏差(m/s)')
    ylabel('展開失敗した回数')
    
    % x軸の目盛りラベルを設定します
    set(gca, 'XTickLabel', labels);
end