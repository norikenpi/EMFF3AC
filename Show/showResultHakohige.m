%hakohigehakohigeで表示
function showResultHakohige(data1, data2, data3, data4, data5, data6, labels)
    figure;
    all_data = [data1, data2, data3, data4, data5, data6];
    group = [ones(size(data1)), 2*ones(size(data2)), 3*ones(size(data3)), 4*ones(size(data4)), 5*ones(size(data5)), 6*ones(size(data6))];

    % 箱ひげ図を描きます
    boxplot(all_data, group, 'Labels', labels , 'Symbol','');
    
    % figureを作成します
    %figure;

    % 各データセットに対する箱ひげ図を作成します


    
    % グラフのタイトルと軸ラベルを付けます
    %title('速度の標準偏差と収束時間の関係')
    %xlabel('ペア決め手法')
    ylabel('展開にかかった時間(s)')
    ylim([300, 650])
end