%hakohigehakohigeで表示
function showResultHakohige(result_velocity5105_plus, result_velocity104_plus, result_velocity5104_plus, labels)
    figure;
    all_data = [result_velocity5105_plus, result_velocity104_plus, result_velocity5104_plus];
    group = [ones(size(result_velocity5105_plus)), 2*ones(size(result_velocity104_plus)), 3*ones(size(result_velocity5104_plus))];

    % 箱ひげ図を描きます
    boxplot(all_data, group, 'Labels', labels , 'Symbol','');
    
    % figureを作成します
    %figure;

    % 各データセットに対する箱ひげ図を作成します


    %{
    % グラフのタイトルと軸ラベルを付けます
    title('速度の標準偏差と収束時間の関係')
    xlabel('速度の標準偏差')
    ylabel('時間(s)')
    %}
end