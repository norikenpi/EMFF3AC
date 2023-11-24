disp("不等式")
% 不等式制約1(全ての入力は最大入力以下)
disp(min(b1 - A1*u_I))

% 不等式制約3 (ノミナル軌道に対する変化量はδ trust region) dispはδになるはず。
disp(min(b3 - A3*u_I))

disp("等式")
% 等式制約2 (最終状態固定)
disp(min(beq - Aeq*u_I))