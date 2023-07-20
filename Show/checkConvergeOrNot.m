function new_array = checkConvergeOrNot(array)
    new_array = zeros(size(array));
    
    % 新しい配列の要素を設定
    new_array(array > 0) = 1;