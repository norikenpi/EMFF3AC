function freq_idx = findFrequencyIndex(idx, pair_idx, param)
    min_idx = min(idx, pair_idx);

    if min_idx > 1
        numbers = (param.N - min_idx + 1):(param.N-1);
        freq_idx1 = sum(numbers);
    elseif min_idx == 1
        freq_idx1 = 0;
    end

    freq_idx = abs(idx - pair_idx) + freq_idx1;
end