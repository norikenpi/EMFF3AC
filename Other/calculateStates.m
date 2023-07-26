%showDataBar(data)で図示
function data = calculateStates(param)
    reative_velocity_square_mean_data = zeros(1,100);
    energy_sum_data = zeros(1,100);
    C1_max_data = zeros(1,100);
    distance_mean_data = zeros(1,100);
    target_distace_mean_data = zeros(1,100);
    C_max_distance = zeros(1,100);
    for i = 1:100
        param.j = i;
        satellites = setInitialSatelliteStates(param);
        %{
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
        %}

        %重心に対する相対速度の最大
        reative_velocity_square_mean_data(i) = calculateReativeVelocitySquareMax(satellites, param);
        %エネルギーの大きさの最大
        energy_sum_data(i) = calculateEnergyMax(satellites, param);
        %重心から見たC1の最大
        C1_max_data(i) = calculateC1Max(satellites, param);
        %重心からの距離の最大
        distance_mean_data(i) = calculateDistanceMax(satellites, param);
        %目標位置までの距離の最大
        target_distace_mean_data(i) = calculateTargetDistanceMax(satellites, param);

        %C1最大とペアを組んでいる衛星との距離
        C_max_distance(i) = calculateC1MaxPairRelativeDistance(satellites, param);

    
        %data = [reative_velocity_square_mean_data; energy_sum_data; C1_sum_data; distance_mean_data; target_distace_mean_data];
        data = [C1_max_data; C_max_distance; C_max_distance; C_max_distance; C_max_distance];
    end
    
end