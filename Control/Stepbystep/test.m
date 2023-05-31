a = {[[1,2];[5,9]];
[6,10];
[7, 11];
};

[row, col] = find(a{3} == 1);

if isempty(col)
    disp("null")
end
disp(row)
disp(col)
disp(a{3}(row, col))