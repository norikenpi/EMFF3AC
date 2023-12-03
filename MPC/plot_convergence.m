
data1 = [35;I_max_list1(1:10)];
data2 = [35;I_max_list2(1:10)*10];
% 折れ線グラフの作成
figure; % 新しい図を開く
plot(data1); % データをプロット
hold on
plot(data2); % データをプロット

% 凡例の追加
legend('普通のfar-field', '片方の磁気モーメントの向きを固定したfar-field');

% オプショナル: グラフのカスタマイズ
xlabel('最適化回数'); % X軸のラベル
ylabel('巻き数100の1U衛星の最大電流(A)(位置trust region 0.05)'); % Y軸のラベル
title('最大電流の収束グラフ'); % グラフのタイトル