%入力したseedの初期状態のパラメータを表示
%重心からの距離
%速度
%重心方向の速度
%エネルギー

function showSatelliteInitialState(seed)
    param = setSimulationParameters();
    param.t = 0.1; 
    %param.freq_all = false;
    param.j = seed;
    %param.pair_type = 'all_energy';
    satellites = setInitialSatelliteStates(param);
    
    position = zeros(1,param.N);
    C1 = zeros(1,param.N);
    velocity = zeros(1,param.N);
    target_error = zeros(1,param.N);
    energy = zeros(1,param.N);

    for i = 1:param.N
        position(i) = norm(satellites{i}.position);
        C1(i) = satellites{i}.velocity(1) - 2 * param.n * satellites{i}.position(3); 
        velocity(i) = norm(satellites{i}.velocity) * dot(satellites{i}.velocity, satellites{i}.position)/norm(dot(satellites{i}.velocity, satellites{i}.position));
        target_error(i) = norm(satellites{i}.position - satellites{i}.position_d);
        energy(i) = (norm(satellites{i}.velocity)^2)/2 + (param.n*satellites{i}.position(2)^2)/2 - (3*param.n*satellites{i}.position(3)^2)/2;
    end
    
    % X軸の要素ラベル
    labels = {' 衛星1', '衛星2', '衛星3', '衛星4', '衛星5', '衛星6', '衛星7', '衛星8'};

    % 新しいfigureを作成
    figure;
    title('データ2');

    % 3つのグラフを縦に並べて描画
    % 1つ目のグラフ
    subplot(5, 1, 1);
    bar(position);
    title(sprintf("Seed = %d", seed));
    ylabel("重心からの距離");
    xticks(1:8);
    %xticklabels(labels);
    grid on;

    % 2つ目のグラフ
    subplot(5, 1, 2);
    bar(C1);
    ylabel('C1');
    xticks(1:8);
    xticklabels(labels);
    grid on;
    
    % 3つ目のグラフ
    subplot(5, 1, 3);
    bar(velocity);
    ylabel('速度');
    xticks(1:8);
    xticklabels(labels);
    grid on;

    % 4つ目のグラフ
    subplot(5, 1, 4);
    bar(target_error);
    ylabel('目標位置誤差');
    xticks(1:8);
    xticklabels(labels);
    grid on;

    % 5つ目のグラフ
    subplot(5, 1, 5);
    bar(energy);
    ylabel('エネルギー');
    xticks(1:8);
    xticklabels(labels);
    grid on;

end
