function createAnimation(histories, param, satellites)
    fps = 30;
    pause = 600;
    dateformat = 'yyyy-MM-dd-HH-mm-ss';
    filename = sprintf('C:/Users/masam/lab/30_simscape/20_磁石/EMFF3/movie/dynamics2record_%s.avi', datetime('now','Format', dateformat));

    writerObj = VideoWriter(filename);
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
        h_traj = cell(1, length(histories.position_histories));
        h_force = cell(1, length(histories.position_histories));
        [h1, h4, h5] = plotSatelliteTrajectory(histories, param, i, colors(1, :), sprintf('Satellite %d', 1), 1);
        for j = 2:length(histories.position_histories)
            [h_traj{j}, h_force{j}] = plotSatelliteTrajectory(histories, param, i, colors(j, :), sprintf('Satellite %d', j), j);
        end

        dim = [0.65 0.5 0.3 0.3];
        str = append('質量 ', string(m), 'kg ','衛星半径', string(a), 'm');
        annotation('textbox',dim,'String',str,'FitBoxToText','on')

        
        axis([-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm,-param.axis_norm,param.axis_norm])
        axis square 
        grid on
        xlabel('X(m)（進行方向）');
        ylabel('Y(m)（面外方向）');
        zlabel('Z(m)（地心方向）');
        set(gca,'YDir','reverse')
        set(gca,'ZDir','reverse')
        str = append('(', string(pause), ' times speed)');
        title(append('Satellite Relative Motion with Trajectory ',str))
        legend([h1, h4, h5], 'Satellite trajectory', 'Satellite force', 'magnetic moment');
        drawnow;

        frame = getframe(fig);
        writeVideo(writerObj, frame);
        hold off
    end
    close(writerObj);
end