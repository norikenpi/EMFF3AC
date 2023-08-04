function weight_distance = calculateEnergyMaxWeghtDistance(param)
    param.t = 0.1;
    [satellites, ~ , ~] = simulateSatellite(param);
    E_max = 0; %Kinetic Energy
    i_max = 0;
    
    for i = 1:param.N
        %KE = norm(satellites{i}.velocity)/2; 
        E = abs((norm(satellites{i}.velocity)^2)/2 + (param.n*satellites{i}.position(2)^2)/2 - (3*param.n*satellites{i}.position(3)^2)/2);
        if E > E_max
            E_max = E;
            i_max = i;
        end
    end

    norm_vector = sqrt(sum(satellites{i_max}.magnetic_moment.^2, 1));%磁気モーメントの大きさに変換
    nonzero_indices = find(norm_vector ~= 0); %使われている周波数インデックス
    num = length(nonzero_indices);%使われている周波数インデックスの数
    norm_vector2 =  norm_vector(nonzero_indices); %%磁気モーメントの大きさが0出ない要素を抽出

    %正規化して，重みに変換
    total_sum = sum(norm_vector2);
    vector_weight = norm_vector2 / total_sum; %正規化して，重みに変換

    %ペアを組んでいる衛星インデックス
    pair_idx = zeros(1, num);
    for i = 1:num
        pair_idx(i) = findFrequencyIndex2index(i_max, nonzero_indices(i));%ペアを組んでいる衛星インデックス

    end
    
    %対応する衛星との距離
    distance = zeros(1,num);
    %disp("nlk")
    %disp(distance)
    for i = 1:num
        %disp(i)
        distance(i) = norm(satellites{i_max}.position - satellites{pair_idx(i)}.position);%対応する衛星との距離
    end
    %disp("nlk")
    %disp(pair_idx)

    weight_distance = sum(vector_weight .* distance);
end