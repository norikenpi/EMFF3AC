%目標相対距離になった時の相対位置と相対速度を計算
assignin('base', 'x1', histories.position_histories(:,:,1));
assignin('base', 'x2', histories.position_histories(:,:,2));

assignin('base', 'v1', histories.velocity_histories(:,:,1));
assignin('base', 'v2', histories.velocity_histories(:,:,2));

x2x1 = x2 - x1; 
v2v1 = v2 - v1;

x2x1_n = vecnorm(x2x1, 2, 2);

for i = 1:size(x2x1, 1)
    if x2x1_n(i) > param.satellite_desired_distance
        disp(i)
        disp(v2v1(i, :))
        break
    end
end