function makeResultFile(param, type, myArray)
    %final = size(histories.position_histories, 1)/10;
    %データを入れるフォルダを作る。
    mkdir(param.path_data)
    filename_txt = strcat(param.path_data, sprintf('/%s_result_%s.txt', param.date, type));
    fileID = fopen(filename_txt, 'w');

    fprintf(fileID, '%d ', myArray);
    fclose(fileID);
end