function average = calculateFunctionAverage(start, finish, interval, w1, w2)
    % start: 区間の開始点
    % finish: 区間の終了点
    % interval: 刻み幅

    % 時間の範囲を作成
    t = start:interval:finish;

    % 関数の計算（ここではsin(t)を例とします）
    y = sin(w1 * t) .* sin(w2 * t);

    % 区間の平均を計算
    average = mean(y);
end
