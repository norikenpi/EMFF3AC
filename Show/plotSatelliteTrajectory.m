% この関数は、与えられた衛星の位置履歴と力履歴に基づいて、衛星の軌道と力ベクトルをプロットし、そのハンドルを返します。
%jは衛星を表す
%iは時間を表す
function [h_traj, h_force, h_pair] = plotSatelliteTrajectory(histories, param, time_i, color, name, satellite_j)
    h_traj = plot3(histories.position_histories(1:time_i, 1, satellite_j), histories.position_histories(1:time_i, 2, satellite_j), histories.position_histories(1:time_i, 3, satellite_j),'LineWidth',1,'Color', [0.7 0.7 0.7]);

    hold on;
    disp(time_i)
    plot3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), 'o', 'MarkerSize', 10, 'MarkerFaceColor', color, 'DisplayName', name);
    plot3(histories.position_d_histories(time_i, 1, satellite_j), histories.position_d_histories(time_i, 2, satellite_j), histories.position_d_histories(time_i, 3, satellite_j), 'o', 'MarkerSize', 5, 'MarkerFaceColor', color, 'DisplayName', name);
    
     
    

    if param.freq_all == true
        for idx = 1:param.N
            if idx ~= satellite_j
                satellite_j2 = idx;
                h_pair = plotLine3D(histories.position_histories(time_i, :, satellite_j), histories.position_histories(time_i, :, satellite_j2));
            end
        end
    elseif param.freq_all == false              
        satellite_j2 = histories.pair_idx(time_i, satellite_j);
        disp(satellite_j)
        disp(satellite_j2)
        h_pair = plotLine3D(histories.position_histories(time_i, :, satellite_j), histories.position_histories(time_i, :, satellite_j2));
    end
    
    
    
    h_force = quiver3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), histories.force_histories(time_i, 1, satellite_j)*param.force_arrow_scale, histories.force_histories(time_i, 2, satellite_j)*param.force_arrow_scale, histories.force_histories(time_i, 3, satellite_j)*param.force_arrow_scale);
    %h_vel = quiver3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), histories.magnetic_moment_histories(time_i, 1, satellite_j)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(time_i, 2, satellite_j)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(time_i, 3, satellite_j)*param.magnetic_moment_arrow_scale);
    h_vel = quiver3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), histories.velocity_histories(time_i, 1, satellite_j)*param.velocity_arrow_scale, histories.velocity_histories(time_i, 2, satellite_j)*param.velocity_arrow_scale, histories.velocity_histories(time_i, 3, satellite_j)*param.velocity_arrow_scale);
    h_force.Color = 'green';
    %h_vel.Color = 'red';
    h_vel.Color = 'red';
    
end
