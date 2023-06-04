function pair_satellite_idx = selectSatellitePair(i, time, param)


    %現在何周期目か
    T_counter = int64(time/param.pair_time);
    remainder = int64(mod(T_counter, param.N) + 1);
    [row, col] = find(param.timetable(remainder) == i);
    % 時間に応じてくじの数字をチェック



    if col == 1
        pair_satellite_idx = param.timetable(row, 2);
    elseif col == 2
        pair_satellite_idx = param.timetable(row, 1);
    elseif isempty(col)
        pair_satellite_idx = i;
    end

    



    %{
    if remainder == 0
        pair_satellite_idx = checkNumber(i, remainder, param.timetable);
        result = (time == 3 || time == 6 || time == 9);
    elseif remainder == 1
        result = (time == 1 || time == 4 || time == 7 || time == 10);
    elseif remainder == 2
        result = (time == 2 || time == 5 || time == 8);
    else
        result = false; % 上記の条件に当てはまらない場合はfalseを返す
    end
    %}