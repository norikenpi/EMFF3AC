function result = lotteryDraw(time)
    % 3n秒のときに1か5が出たらtrue
    % 3n+1秒のときに2か4が出たらtrue
    % 3n+2秒のときに3か6が出たらtrue
    % それ以外ではfalseを返す

    % くじの数字
    numbers = [1 2 3 4 5 6];

    % ランダムにくじを引く
    rng('shuffle');  % 乱数ジェネレータのシードをシステムクロックに基づいて変更
    drawnNumber = numbers(randi(numel(numbers)));
    disp(drawnNumber)

    % 引いた数字に応じて条件をチェック
    if (time == 3 && (drawnNumber == 1 || drawnNumber == 5)) || ...
       (time == 4 && (drawnNumber == 2 || drawnNumber == 4)) || ...
       (time == 5 && (drawnNumber == 3 || drawnNumber == 6))
        result = true;
    else
        result = false;
    end
end
