% 20個の整数値の例
data = [14 13 16 14 17 17 13 18 17 14 9 11 10 15 12 15 13 7 17]/20;

% 箱ひげ図を描画
figure;
boxplot(data,0.1);

% y軸の範囲を0から1に設定
ylim([0 1]);

% グラフのタイトルと軸ラベルを設定
title('');
xlabel('C_1 max, m');
ylabel('Ncluster / Ntotal');
