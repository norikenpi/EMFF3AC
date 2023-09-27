function matrix = generateMatrix(N)
    matrix = zeros(N, N-1);

    % First row
    matrix(1, 1) = -1;

    % Middle rows
    for i = 2:N-1
        matrix(i, i-1) = 1;
        matrix(i, i) = -1;
    end

    % Last row
    matrix(N, N-1) = 1;
end

