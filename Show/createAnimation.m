function createAnimation(histories, param, satellites)
    fps = 31;
    pause = param.pause;
    filename_option = "";
    %final = size(histories.position_histories, 1)/10;


    %データを入れるフォルダを作る。
    mkdir(param.path_data)
    filename_movie = strcat(param.path_data, sprintf('/%s_movie%s.avi', param.date, filename_option));
    filename_var = strcat(param.path_data, sprintf('/%s_var.mat', param.date));

    
    
    %matファイルに全Fてのワークスペース変数の保存
    save(filename_var);

    writerObj = VideoWriter(filename_movie);
    writerObj.Quality = 100;
    writerObj.FrameRate = fps;
    open(writerObj);

    figcolor = [1 1 1];
    fig = figure('color', figcolor);

    m = param.mass;
    a = param.radius;

    colors = jet(size(histories.position_histories, 3)); % N個の衛星に対して異なる色を設定

    for i = 1:round(pause/(fps*param.dt)):abs(param.finished_time)/param.dt
    
        disp(i)
        %h_traj = cell(1, length(histories.position_histories));
        %h_force = cell(1, length(histories.position_histories));
        [h1, h4, h5, h_pair] = plotSatelliteTrajectory(histories, param, i, colors(1, :), sprintf('Satellite %d', 1), 1);
        
        for j = 2:size(histories.position_histories, 3)
            [h_traj, h_force, h_mag, h_pair] = plotSatelliteTrajectory(histories, param, i, colors(j, :), sprintf('Satellite %d', j), j);
        end
        
        %x方向からの視点
        dim = [0.65 0.5 0.3 0.3];
        path_parent = append('質量 ', string(m), 'kg ','衛星半径', string(a), 'm');
        annotation('textbox',dim,'String',path_parent,'FitBoxToText','on')

        
        axis([-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm])
        axis square 
        grid on
        xlabel('X(m)（進行方向）');
        ylabel('Y(m)（面外方向）');
        zlabel('Z(m)（地心方向）');
        set(gca,'YDir','reverse')
        set(gca,'ZDir','reverse')
        path_parent = append('(', string(pause), ' times speed)');
        title(append('Satellite Relative Motion with Trajectory ',path_parent))
        %legend([h1, h4, h5], 'Satellite trajectory', 'Satellite force', 'magnetic moment');
        legend(h_pair, 'Pair satellite');
        
        %グラフ画像に日付を入れる。
        xPosition = xlim*3; % x軸の範囲を取得
        yPosition = ylim; % y軸の範囲を取得

        yPosition2 = ylim/2;
        text(xPosition(2), yPosition(2), string(param.date), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');

        %時間を表示
        text(xPosition(2), yPosition2(2), string(i*param.dt), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
        drawnow;

        frame = getframe(fig);
        writeVideo(writerObj, frame);
        hold off
    end
    close(writerObj);


    %パラメータをテキストファイル化
    filename_param = strcat(param.path_data, '/param.txt');
    outputStructToTextFile(param, filename_param)
end