function plot_s(s, num, N, rr)
    % 2衛星の動画を表示。
    %3次元座標
    data = reorderMatrix2(s);
    satelllites = cell(1, num);

    % 衛星インスタンス生成
    for i = 1:num
        satelllites{i} = zeros(N, 3);
    end
    
    % 衛星データ格納
    for i = 1:N
        for j = 1:num
            satelllites{j}(i,:) = data(3*num*(i-1)+3*(j-1)+1:3*num*(i-1)+3*(j-1)+3).';
        end
    end
    
    % ビデオライターオブジェクトの作成
    v = VideoWriter('points_motion_3D.avi'); % AVIファイル形式で動画を保存
    % 画質の設定（例：品質を最大に）
    v.Quality = 100;
    open(v);
    
    % フィギュアの作成
    figure;
    axis equal;
    xlim([-0.3, 0.3]); % x軸の範囲を調整
    ylim([-0.3, 0.3]); % y軸の範囲を調整
    zlim([-0.3, 0.3]); % z軸の範囲を調整
    hold on;
    grid on; % グリッドを表示
    
    set(gca, 'ZDir', 'reverse')
    
    % 軸のラベルを設定
    xlabel('X[m](軌道進行方向)');
    ylabel('Y[m](軌道面垂直方向)');
    zlabel('Z[m](地心方向)');
    
    theta = linspace(0, 2 * pi, 100); % 0から2πまでの角度を生
    colors = hsv(num); % HSVカラースペースを使用してN個の異なる色を生成
    
    % 各フレームでの点の位置をプロットし、そのフレームを動画に書き込む
    for i = 1:5:N
        cla;
        
        for j = 1:length(rr)
            % レコード盤軌道をプロット
            x1 = -2*rr(j)*cos(theta); % x座標を計算
            y1 = sqrt(3)*rr(j)*sin(theta); % y座標を計算
            z1 = rr(j)*sin(theta);
            plot3(x1, y1, z1, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1); % 円を灰色で描画
        end

        % x軸方向の点線を描画
        x_position = -1:0.1:1; % x軸方向の点線のx座標を指定
        y_position = zeros(size(x_position)); % y座標はすべて0に設定
        plot(x_position, y_position, 'k--', 'LineWidth', 1.5); % 点線を描画
       
        for j = 1:num
            % 軌道をプロット
            plot3(satelllites{j}(1:N,1), satelllites{j}(1:N,2), satelllites{j}(1:N,3), '-', 'Color', colors(j,:));
            % 衛星をプロット
            plot3(satelllites{j}(i,1), satelllites{j}(i,2), satelllites{j}(i,3), '.', 'MarkerSize', 15, 'Color', colors(j,:));
            % 衛星の初期値をプロット
            plot3(satelllites{j}(1,1), satelllites{j}(1,2), satelllites{j}(1,3), 'o', 'MarkerSize', 5, 'Color', colors(j,:));
        end
        % 視点を変更
        azimuth = 225; % 方位角
        elevation = 30; % 仰角
        view(azimuth, elevation);
    
        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    
    % ビデオの保存
    close(v);
    
end


function B = reorderMatrix2(A)
    idx = find(mod(1:length(A), 6) == 1 | mod(1:length(A), 6) == 2 | mod(1:length(A), 6) == 3);
    A = flip(A(idx));
    B = [];
    %A = 1:100;
    n = 3;
    for i = 1:n:length(A)
        sub_vector = A(i:min(i+n-1, length(A)));
        B = [B; flip(sub_vector)];
    end
end