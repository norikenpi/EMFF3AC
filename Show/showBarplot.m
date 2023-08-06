function showBarplot(data1, data2, data3)
    figure;

    % 各配列の大きさ（要素数）を取得します
    dataSizes = [length(data1), length(data2), length(data3)];
    
    % 棒グラフを作成します
    bar(dataSizes);
    
    % グラフのタイトルと軸ラベルを付けます
    title('Size of Data Sets')
    xlabel('Data Set')
    ylabel('Size')
    
    % x軸の目盛りラベルを設定します
    set(gca, 'XTickLabel', {'Data1', 'Data2', 'Data3'});
end