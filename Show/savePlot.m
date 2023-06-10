function savePlot(param, fileName)

    %グラフ画像に日付を入れる。
    xPosition = xlim; % x軸の範囲を取得
    yPosition = ylim; % y軸の範囲を取得
    text(xPosition(2), yPosition(2), string(param.date), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
   
    %matファイルに全てのワークスペース変数の保存
    filename_var = strcat(param.path_data, sprintf('/%s_var.mat', param.date));
    save(filename_var);

    %パラメータをテキストファイル化
    filename_param = strcat(param.path_data, sprintf('/%s_param.txt', param.date));
    outputStructToTextFile(param, filename_param);

    %データを入れるフォルダを作る。
    mkdir(param.path_data)

    %グラフを保存
    
    filename_fig = strcat(param.path_data, sprintf('/%s_plot_%s.png', param.date, fileName));
    saveas(gcf, fullfile(filename_fig))

    
    
