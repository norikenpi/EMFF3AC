function outputStructToTextFile(structure, filename)
    fid = fopen(filename, 'w'); % ファイルを書き込みモードで開く
    fields = fieldnames(structure); % 構造体のフィールド名を取得
    for i = 1:numel(fields)
        fieldName = fields{i}; % フィールド名を取得
        fieldValue = structure.(fieldName); % フィールドの値を取得
        
        % フィールドの値が数値や文字列の場合
        if isnumeric(fieldValue) || ischar(fieldValue)
            fieldValueStr = num2str(fieldValue); % 数値や文字列を文字列に変換
        elseif islogical(fieldValue)
            fieldValueStr = num2str(double(fieldValue)); % 論理値を文字列に変換
        elseif iscell(fieldValue) % フィールドの値がセル配列の場合
            fieldValueStr = cellfun(@num2str, fieldValue, 'UniformOutput', false); % セル配列の各要素を文字列に変換
            fieldValueStr = strjoin(fieldValueStr, ', '); % カンマで要素を結合
        else
            fieldValueStr = 'Cannot output field value'; % 出力できないフィールドの値
        end
        
        fprintf(fid, '%s = %s\n', fieldName, fieldValueStr); % ファイルにフィールド名と値を書き込み
    end
    fclose(fid); % ファイルを閉じる
end
