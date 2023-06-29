function pair_satellite_idx = selectSatellitePair(i, time, param)

    if param.current_type == "DC"
        %現在何周期目か
        T_counter = int64(time/param.pair_time);
        pair_number = size(param.timetable,1);
    
        %T_counter週目で稼働する衛星群
        row = int64(mod(T_counter, pair_number) + 1);
    
        %その衛星群の中にi番目の衛星は含まれているかを確認
        col = find(param.timetable(row,:) == i);
        % 時間に応じてくじの数字をチェック 
        if col == 1
            pair_satellite_idx = param.timetable(row, 2);
        elseif col == 2
            pair_satellite_idx = param.timetable(row, 1);
        elseif col == 3
            pair_satellite_idx = param.timetable(row, 4);
        elseif col == 4
            pair_satellite_idx = param.timetable(row, 3);
        elseif isempty(col)
            pair_satellite_idx = i;
        end

    elseif param.current_type == "AC"
        %衛星iに対応する衛星をpair_satellite_idxに格納する。
        pair_satellite_idx = param.set{i};
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