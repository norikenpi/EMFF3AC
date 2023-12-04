% 目的関数の係数
f = [1; -1]; % x = x^+ - x^-

% 不等式制約 A * x <= b
A = [-1 -1;   % 絶対値制約 x^+ + x^- >= 3 の線形化
     -1 1]; % x >= 10 の線形化
b = [-12; -10];

% すべての変数の下限値（非負）
lb = [0; 0];

% linprogを使用して問題を解く
options = optimoptions('linprog','Algorithm','dual-simplex');
[x, fval, exitflag, output] = linprog(f, A, b, [], [], lb, [], options);

% 解の取得
x_val = x(1) - x(2); % x = x^+ - x^-

