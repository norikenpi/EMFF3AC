
d = 1;
delta = 0.01;
%delta_dot = 0.01;
Imax = 1;
C = 1;

k = C * Imax^2/(d^4 * delta);

D_1 = [-2, -1;
     1, -1;
     1, 2] * k/4;

D_2 = [-1, 0;
     1, -1;
      0, 1]*k/2;

D_3 = 

%棒一直線
D_4 = generateMatrix(N) * k/4;

N = 3;
DD_ = D_2;

max_eigenvalue = evaluateFormation(N, DD_);
disp(max_eigenvalue)