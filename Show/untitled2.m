% コイルの分割数とコイル間距離による誤差について

figure
distance = [0.15,0.1,0.065];
radius = 0.015;
for i = distance
    
    showCoilDipoleError(i, radius, param)

    hold on

end



grid on
xlabel('電流要素分割数')
ylabel('100分割に対する誤差(%)')
title('直径3cmの電流要素分割数に対する力の精度（衛星間距離別）x方向')
legend(append(string(distance(1)), "m"), append(string(distance(2)), "m"), append(string(distance(3)), "m"))
hold on