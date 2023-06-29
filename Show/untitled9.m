%ダイポールを回転させて方向によってどういう力を発生させられるかを調べる。


m1 = [1; 0; 0];
m2 = [1; 0; 0];
r = [1; 0; 0];
F = far_field(m1, m2, r, param);

total = 100;
delta_theta = 2*pi/total;
theta = 0;
data_vector = zeros(2,total);
for i = 1:total
    theta = theta + delta_theta;
    % 回転行列の作成
    R = [cos(theta) -sin(theta) 0;
         sin(theta)  cos(theta) 0;
            0           0       1];
    
    % ベクトルの回転
    rotated_m2 = R * m2;
    F = far_field(m1, rotated_m2, r, param);
    data_vector(:, i) = F(1:2);
end
figure
scatter(data_vector(1, :), data_vector(2, :), 50, 'filled');
hold on;
%h1 = scatter(0, 0, 50,'k', 'filled');
xlabel('X軸');
ylabel('Y軸');
title('x軸方向に並んだxy平面上にある2つのダイポールが発生させられる力の向きと大きさ');
%xlim([-1, 1]);  % x軸の範囲を-1から1に設定
%ylim([-1, 1]);  % y軸の範囲を-1から1に設定
%legend(h1, 'origin');
grid on;
%set(gca,'YDir','reverse')
daspect([1 1 1]);