disp("不等式")
% 不等式制約1(全ての入力は最大入力以下)
disp(min(b1 - A1*u))

% 不等式制約2 ( (衛星間距離はR以下)
disp(min(b2 - A2*u))

% 不等式制約3 位置(ノミナル軌道に対する変化量はδ trust region) dispはδになるはず。
%disp(min(b3 - A3*u))

% 不等式制約4 磁気モーメント(ノミナル軌道に対する変化量はδ trust region) dispはδになるはず。
%disp(min(b4 - A4*u))

disp("等式")

% 等式制約2 (最終状態固定)
disp(min(beq2 - Aeq2*u))

%相対距離確認
disp("相対距離確認")


