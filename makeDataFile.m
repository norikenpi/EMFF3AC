function makeDataFile(param, i, type)
    %final = size(histories.position_histories, 1)/10;


    %データを入れるフォルダを作る。
    mkdir(param.path_data)
    filename_var = strcat(param.path_data, sprintf('/%s_var_%s_%d.mat', param.date, type, i));

    
    
    %matファイルに全Fてのワークスペース変数の保存
    save(filename_var);




    %パラメータをテキストファイル化
    filename_param = strcat(param.path_data, sprintf('/param_%s_%d.txt', type, i));
    outputStructToTextFile(param, filename_param)
end