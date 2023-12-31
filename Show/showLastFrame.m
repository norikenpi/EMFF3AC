function showLastFrame(histories, param, satellites)
    figcolor = [1 1 1];
    fig = figure('color', figcolor);

    m = satellites{1}.mass;
    a = satellites{1}.radius;
    

    colors = jet(size(histories.position_histories, 3)); % N個の衛星に対して異なる色を設定

    %修了時間
    i = size(histories.position_histories, 1);
    i = int32(abs(param.finished_time)/param.dt);
    

    h_traj = cell(1, length(histories.position_histories));
    h_force = cell(1, length(histories.position_histories));
    [h1, h4, h5] = plotSatelliteTrajectory(histories, param, i, colors(1, :), sprintf('Satellite %d', 1), 1);
    for j = 2:size(histories.position_histories, 3)
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
    title('Final Frame');
    legend([h1, h4, h5], 'Satellite trajectory', 'Satellite force', 'magnetic moment');
    drawnow;
end