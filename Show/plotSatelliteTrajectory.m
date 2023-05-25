% この関数は、与えられた衛星の位置履歴と力履歴に基づいて、衛星の軌道と力ベクトルをプロットし、そのハンドルを返します。
%jは衛星を表す
%iは時間を表す
function [h_traj, h_force, h_mag] = plotSatelliteTrajectory(histories, param, i, color, name, j)
    h_traj = plot3(histories.position_histories{j}(1, 1:i), histories.position_histories{j}(2, 1:i), histories.position_histories{j}(3, 1:i),'LineWidth',1,'Color', [0.7 0.7 0.7]);
    hold on;
    plot3(histories.position_histories{j}(1, i), histories.position_histories{j}(2, i), histories.position_histories{j}(3, i), 'o', 'MarkerSize', 10, 'MarkerFaceColor', color, 'DisplayName', name);
    %forces = cell2mat(force_histories);
    h_force = quiver3(histories.position_histories{j}(1, i), histories.position_histories{j}(2, i), histories.position_histories{j}(3, i), histories.force_histories{j}(1, i)*param.force_arrow_scale, histories.force_histories{j}(2, i)*param.force_arrow_scale, histories.force_histories{j}(3, i)*param.force_arrow_scale);
    h_mag = quiver3(histories.position_histories{j}(1, i), histories.position_histories{j}(2, i), histories.position_histories{j}(3, i), histories.magnetic_moment_histories{j}(1, i)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories{j}(2, i)*param.magnetic_moment_arrow_scale, histories.magnetic_moment_histories{j}(3, i)*param.magnetic_moment_arrow_scale);
    h_force.Color = 'green';
    h_mag.Color = 'red';
end
