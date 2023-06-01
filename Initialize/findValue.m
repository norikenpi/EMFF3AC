function [row, col] = findValue(mat, value)
    linearIndex = find(mat == value);
    [row, col] = ind2sub(size(mat), linearIndex);
end
