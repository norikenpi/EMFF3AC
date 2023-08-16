function arr = loadArrayFromTxt(filename)
    % 引数:
    %   filename: 配列が保存されている.txtファイルの名前
    % 戻り値:
    %   arr: .txtファイルから復元された1次元の配列

    % .txtファイルから数値を読み込む
    fileID = fopen(filename, 'r'); % 'r'は読み取りモード
    arr = fscanf(fileID, '%f').'; % %fは浮動小数点数のフォーマット指定子
    fclose(fileID);
end
