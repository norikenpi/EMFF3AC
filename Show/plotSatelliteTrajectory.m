% この関数は、与えられた衛星の位置履歴と力履歴に基づいて、衛星の軌道と力ベクトルをプロットし、そのハンドルを返します。
%jは衛星を表す
%iは時間を表す
function [h_traj, h_force, h_mag, h_pair] = plotSatelliteTrajectory(histories, param, time_i, color, name, satellite_j)

    h_traj = plot3(histories.position_histories(satellite_j, time_i, 1), histories.position_histories(satellite_j, time_i, 2), histories.position_histories(satellite_j, time_i, 3),'LineWidth',1,'Color', [0.7 0.7 0.7]);
    hold on;
    plot3(histories.position_histories(satellite_j, time_i, 1), histories.position_histories(satellite_j, time_i, 2), histories.position_histories(satellite_j, time_i, 3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', color, 'DisplayName', name);

    satellite_j2 = histories.pair_idx(satellite_j, time_i);
    h_pair = plotLine3D(histories.position_histories(satellite_j, time_i, :), histories.position_histories(satellite_j2, time_i, :));
    
    
    h_force = quiver3(histories.position_histories(satellite_j, time_i, 1), histories.position_histories(satellite_j, time_i, 2), histories.position_histories(satellite_j, time_i, 3), histories.force_histories(satellite_j, time_i, 1)*param.force_arrow_scale, histories.force_histories(satellite_j, time_i, 2)*param.force_arrow_scale, histories.force_histories(satellite_j, time_i, 3)*param.force_arrow_scale);
    h_mag = quiver3(histories.position_histories(satellite_j, time_i, 1), histories.position_histories(satellite_j, time_i, 2), histories.position_histories(satellite_j, time_i, 3), histories.magnetic_moment_histories(satellite_j, time_i, 1)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(satellite_j, time_i, 2)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories(satellite_j, time_i, 3)*param.magnetic_moment_arrow_scale);
    h_force.Color = 'green';
    h_mag.Color = 'red';
    
end
