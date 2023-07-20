function data = calculateStates(param)
    reative_velocity_square_mean_data = zeros(1,100);
    energy_sum_data = zeros(1,100);
    C1_sum_data = zeros(1,100);
    distance_mean_data = zeros(1,100);
    target_distace_mean_data = zeros(1,100);
    for i = 1:100
        param.j = i;
        satellites = setInitialSatelliteStates(param);

        %重心に対する相対速度の2乗平均
        reative_velocity_square_mean_data(i) = calculateReativeVelocitySquareMean(satellites, param);
        %エネルギーの大きさの和
        energy_sum_data(i) = calculateEnergySum(satellites, param);
        %重心から見たC1
        C1_sum_data(i) = calculateC1Sum(satellites, param);
        %重心からの距離の4乗平均
        distance_mean_data(i) = calculateDistanceMean(satellites, param);
        %目標位置までの距離
        target_distace_mean_data(i) = calculateTargetDistanceMean(satellites, param);
    
        data = [reative_velocity_square_mean_data; energy_sum_data; C1_sum_data; distance_mean_data; target_distace_mean_data];
    end
    
end