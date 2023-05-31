
for i=1:20
    [m, n, p] = size(reshape(1:20, 2, 2, 20/4));
    % インデックスiに対応するx, y, z座標を計算
    [ix, iy, iz] = ind2sub([m, n, p], i);
    rng(1)
    x = 0.15 * (ix - (m+1)/2) + rand() * 0.00 - 0.02; % -0.15から0.15の範囲で衛星を配置
    y = 0.15 * (iy - (n+1)/2) + rand() * 0.00 - 0.02;
    z = 0.15 * (iz - (p+1)/2) + rand() * 0.00 - 0.02;
    
    disp(i)
    disp([x;y;z])
end