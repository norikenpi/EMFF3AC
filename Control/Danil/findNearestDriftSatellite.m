% 衛星の配列と、最も近い衛星を見つけたい衛星のインデックス、C1の値、C1の最小値を受け取り、最も近いC1値がC1_minより大きくなるような最近傍衛星を探索します。除外する衛星を更新しながら探索を続けます。
function [nearest_drift_satellite_idx, C1, relative_position, converge, relative_position_min] = findNearestDriftSatellite(satellites, param, target_idx)
    % 初期化
    excluded_idx = []; % 除外する衛星のインデックスを保持する配列を初期化
    nearest_drift_satellite_idx = findNearestSatellite(satellites, target_idx, excluded_idx); % 指定した衛星以外で最も近い衛星のインデックスを取得
    target_satellite = satellites{target_idx}; 
    disp(target_idx)
    disp(nearest_drift_satellite_idx)
    nearest_satellite = satellites{nearest_drift_satellite_idx}; % 最も近い衛星の座標を取得
    relative_position = - nearest_satellite.position + target_satellite.position; % 相対位置を計算
    relative_position_min = relative_position;
    relative_velocity = - nearest_satellite.velocity + target_satellite.velocity; % 相対速度を計算    
    C1 = relative_velocity(1)/param.n - 2*relative_position(3); % C1値を計算

    

    % C1の最小値を満たす最も近い衛星を探索
    converge = 0;
    if norm(relative_position_min) > param.safety_distance
        while norm(C1) <= param.C1_min % C1の最小値より小さい間、探索を続ける
            excluded_idx = [excluded_idx, nearest_drift_satellite_idx]; % 除外する衛星のインデックスに、現在の最近傍衛星を追加
            if size(excluded_idx, 2) == param.N-1
                converge = 1;
                disp("no satellite")
                break
            end
            nearest_drift_satellite_idx = findNearestSatellite(satellites, target_idx, excluded_idx); % 除外した衛星を除いた最も近い衛星を探索
            target_satellite = satellites{target_idx}; % 最も近い衛星の座標を取得
            nearest_satellite = satellites{nearest_drift_satellite_idx}; % 最も近い衛星の座標を取得
            relative_position = nearest_satellite.position - target_satellite.position; % 相対位置を計算
            relative_velocity = nearest_satellite.velocity - target_satellite.velocity; % 相対速度を計算
            C1 = relative_velocity(1)/param.n - 2*relative_position(3); % C1値を計算
        end
    end


    
end

