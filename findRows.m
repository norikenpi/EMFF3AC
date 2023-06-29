%function freq_idx =searchPairFrequencyIndex(i, pair_satellite_idx);

function output = findRows(searchElement1, searchElement2, param)
    matrix = param.timetable;
    for i = 1:size(matrix, 1)
        if ismember(searchElement1, matrix(i, :)) && ismember(searchElement2, matrix(i, :))
            output = i;
        end
    end
end
