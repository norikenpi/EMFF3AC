% コイルの分割数とコイル間距離による誤差について
%satelliteAverageForceTotalonly2で磁場計算方法を書く

figure
distance = [1,0.3,0.15];
radius = param.radius;
for i = distance
    
    showCoilDipoleError(i, radius, param)

    hold on

end



grid on
xlabel('電流要素分割数')
ylabel('100分割に対する誤差(%)')
title(sprintf('直径%dmの電流要素分割数に対する力の精度（衛星間距離別）x方向', param.radius))
legend(append(string(distance(1)), "m"), append(string(distance(2)), "m"), append(string(distance(3)), "m"))
hold on