function createAnimation(histories, param, satellites)
    fps = 31;
    pause = 300;
    dateformat = 'yyyy-MM-dd-HH-mm-ss';
    date = datetime('now','Format', dateformat);

    path_data = sprintf(strcat(param.path, '/movie/%s'), date);

    %データを入れるフォルダを作る。
    mkdir(path_data)
    filename_movie = strcat(path_data, '/movie.avi');

    writerObj = VideoWriter(filename_movie);
    writerObj.Quality = 100;
    writerObj.FrameRate = fps;
    open(writerObj);

    figcolor = [1 1 1];
    fig = figure('color', figcolor);

    m = satellites{1}.mass;
    a = satellites{1}.radius;

    colors = jet(length(histories.position_histories)); % N個の衛星に対して異なる色を設定

    for i = 1:round(pause/(fps*param.dt)):size(histories.position_histories{1}, 2)
    
        disp(i)
        %h_traj = cell(1, length(histories.position_histories));
        %h_force = cell(1, length(histories.position_histories));
        [h1, h4, h5, h_pair] = plotSatelliteTrajectory(histories, param, i, colors(1, :), sprintf('Satellite %d', 1), 1);
        
        for j = 2:length(histories.position_histories)
            plotSatelliteTrajectory(histories, param, i, colors(j, :), sprintf('Satellite %d', j), j);
        end

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
        drawnow;

        frame = getframe(fig);
        writeVideo(writerObj, frame);
        hold off
    end
    close(writerObj);


    %パラメータをテキストファイル化
    filename_param = strcat(path_data, '/param.txt');
    outputStructToTextFile(param, filename_param)
end