function arr = createArrayFromTextFile(filePath)
    % ファイルを開く
    fileID = fopen(filePath, 'r');
    
    % エラーチェック
    if fileID == -1
        error('Cannot open file: %s', filePath);
    end

    % fscanf関数を使用してファイルからデータを読み取る
    arr = fscanf(fileID, '%f');
    
    % 1x100の配列形状に変形
    arr = reshape(arr, [1, 100]);

    % ファイルを閉じる
    fclose(fileID);
end