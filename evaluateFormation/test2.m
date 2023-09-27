B = rand(3,3);
for i = 1:0.1:10
    k = i;
    
    A = [zeros(3), eye(3);
         k*B, k*B];
    eigen_value = eig(A);
    disp(max(eigen_value))
end
