function pair_idx = findFrequencyIndex2index(idx, freq_idx)
    %周波数インデックスと片方の衛星番号から，もう片方の衛星番号を調べる．
    if freq_idx - 7 <= 0
        i = 1;
        j = freq_idx - 1 + 2;
    elseif freq_idx - 13 <= 0
        i = 2;
        j = freq_idx - 8 + 3;
    elseif freq_idx - 18 <= 0
        i = 3;
        j = freq_idx - 14 + 4;
    elseif freq_idx - 22 <= 0
        i = 4;
        j = freq_idx - 19 + 5;
    elseif freq_idx - 25 <= 0
        i = 5;
        j = freq_idx - 23 + 6;
    elseif freq_idx - 27 <= 0
        i = 6;
        j = freq_idx - 26 + 7;
    elseif freq_idx - 28 <= 0 
        i = 7;
        j = freq_idx - 28 + 8;
    end

    if i == idx
        pair_idx = j;
    elseif j == idx
        pair_idx = i;
    end