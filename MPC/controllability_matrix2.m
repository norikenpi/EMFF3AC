function mat = controllability_matrix2(A, N)
    mat = [];
    for j = 1:N
        mat_i = A^(N - j + 1);
        mat = [mat;mat_i];
    end

end