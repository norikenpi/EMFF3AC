% この関数は、与えられた衛星の位置履歴と力履歴に基づいて、衛星の軌道と力ベクトルをプロットし、そのハンドルを返します。
%jは衛星を表す
%iは時間を表す
function [h_traj, h_force, h_mag, h_pair] = plotSatelliteTrajectory(histories, param, time_i, color, name, satellite_j)
    h_traj = plot3(histories.position_histories(1:time_i, 1, satellite_j), histories.position_histories(1:time_i, 2, satellite_j), histories.position_histories(1:time_i, 3, satellite_j),'LineWidth',1,'Color', [0.7 0.7 0.7]);
    hold on;
    plot3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), 'o', 'MarkerSize', 10, 'MarkerFaceColor', color, 'DisplayName', name);


    satellite_j2 = histories.pair_idx(time_i, satellite_j);
    h_pair = plotLine3D(histories.position_histories(time_i, :, satellite_j), histories.position_histories(time_i, :, satellite_j2));
    
    
    h_force = quiver3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), histories.force_histories(time_i, 1, satellite_j)*param.force_arrow_scale, histories.force_histories(time_i, 2, satellite_j)*param.force_arrow_scale, histories.force_histories(time_i, 3, satellite_j)*param.force_arrow_scale);
    h_mag = quiver3(histories.position_histories(time_i, 1, satellite_j), histories.position_histories(time_i, 2, satellite_j), histories.position_histories(time_i, 3, satellite_j), histories.magnetic_moment_histories(time_i, 1, satellite_j)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(time_i, 2, satellite_j)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(time_i, 3, satellite_j)*param.magnetic_moment_arrow_scale);
    h_force.Color = 'green';
    h_mag.Color = 'red';
    
end

