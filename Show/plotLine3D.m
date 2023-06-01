function h = plotLine3D(point1, point2)
    % point1: 最初の点の座標 [x1; y1; z1]
    % point2: 最後の点の座標 [x2; y2; z2]

    % x座標の配列
    x = [point1(1), point2(1)];
    
    % y座標の配列
    y = [point1(2), point2(2)];
    
    % z座標の配列
    z = [point1(3), point2(3)];

    % 点と線を図示
    h = plot3(x, y, z,'LineWidth',1,'Color', [0.1 0.1 0.8]);
    
end
