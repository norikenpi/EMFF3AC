% シミュレーションパラメータを設定する関数。サンプリング間隔、シミュレーション時間、比例ゲイン、微分ゲイン、最大力などのパラメータを設定します。
function param = setSimulationParameters()
    myu = 3.986*10^14;
    earth_radius = 6378.140*10^3; % 地球半径
    altitude = 500*10^3; % 高度
    r_star = earth_radius + altitude; % 地心からの距離
    param.n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度

    param.approximation = 'trapezoid';
    param.dt = 0.01; % シミュレーションタイムステップ
    param.time_step = 0.1; % 制御タイムステップ
    param.t = 1; % シミュレーション時間
    param.initial_error = 0.005;%初期誤差
    param.satellite_initial_distance = 0.065; %初期衛星間距離
    param.satellite_desired_distance = 0.15; %衛星間距離

    %保存先
    param.path = 'C:/Users/masam/lab/30_simscape/20_磁石/';
    %param.path = 'C:/Users/nakan/OneDrive/デスクトップ/kubota';
    param.pair_time = 1; % ペアリングされてる時間

    

    %Danil式パラメータ
    param.Kp = 10^(-6); % 比例ゲイン10^(-6)
    param.Kd = 0.05; % 微分ゲイン
    param.C1_min = 10^-7; % 最小ドリフト
    param.j = 9; %シード値
    param.C1_ini = 0.1;%初期C1の最大値
    param.safety_distance = 0.05; % 衝突回避制御を実施するための距離
    param.avoid_collision_magnetic_moment = 0.00005; % 衝突回避制御を行うときの磁気ダイポールゲイン
    param.max_distance = 1; % 通信可能距離
    param.min_distance_nopair = 0.05; % ペアリングした衛星よりも近くに別の衛星があるとき、この距離以内にある場合、制御しない。

    %衛星の初期パラメータ
    param.angular_velocity = [0; 0; 0.1]; % 角速度
    param.magnetic_moment = [0; 0.01; 0]; % 磁気モーメント
    param.mass = 0.03; % 衛星質量
    param.moment_of_inertia = 1; % 慣性モーメント

    %2018年の野田さんの宇科連準拠
    param.coilN = 17; % 巻き数
    param.I_max = 1; % 最大電流
    param.radius = 0.015; %衛星半径
    param.max_magnetic_moment = param.coilN * param.I_max * pi * param.radius^2; % 最大磁気モーメント
    

    param.N = 16; % 衛星の数

    %紐ありの場合
    param.cof = 0.8; %反発係数
    param.length = 0.17;

    %{
    param.timetable = {[5,9];
                       [6,10];
                       [7,11];
                       [8,12];
                       [9,13];
                       [10,14];
                       [11,15];
                       [12,16]
                       [1,5];
                       [2,6];
                       [3,7];
                       [4,8];
                       [[1,2];[9,10]];
                       [[2,4];[10,12]];
                       [[3,4];[11,12]];
                       [[1,3];[9,11]];
                       [[5,6];[13,14]];
                       [[6,8];[14,16]];
                       [[7,8];[15,16]];
                       [[5,7];[13,15]]};
    %}
    %{
    param.timetable = [[5,9];
                       [6,10];
                       [7,11];
                       [8,12];
                       [9,13];
                       [10,14];
                       [11,15];
                       [12,16]
                       [1,5];
                       [2,6];
                       [3,7];
                       [4,8];
                       [1,2];
                       [9,10];
                       [2,4];
                       [10,12];
                       [3,4];
                       [11,12];
                       [1,3];
                       [9,11];
                       [5,6];
                       [13,14];
                       [6,8];
                       [14,16];
                       [7,8];
                       [15,16];
                       [5,7];
                       [13,15]];
    %}
    %{
    param.timetable = {[1,2];
                       [3,4];
                       [1,3];
                       [2,4]};
    %}

    param.timetable = [[[1,9],[4,12]];
                       [[2,10],[0,0]];
                       [[3,11],[0,0]];
                       [[9,13],[12,16]];
                       [[10,14],[0,0]];
                       [[11,15],[0,0]];
                       [[5,13],[8,16]];
                       [[6,14],[0,0]];
                       [[7,15],[0,0]];
                       [[1,2],[5,6]];
                       [[2,3],[6,7]];
                       [[3,4],[7,8]];
                       [[9,10],[0,0]];
                       [[10,11],[0,0]];
                       [[11,12],[0,0]];
                       [[13,14],[0,0]];
                       [[14,15],[0,0]];
                       [[15,16],[0,0]]];


    

    % 紐を使ったときに使うパラメータ
    % 各衛星と接続している衛星
    param.set =  {[2,9];
                   [1,3,10];
                   [2,4,11];
                   [3,12];
                   [6,13];
                   [5,7,14];
                   [6,8,15];
                   [7,16];
                   [1,10,13];
                   [2,9,11,14];
                   [3,10,12,15];
                   [4,11,16];
                   [5,9,14];
                   [6,10,13,15];
                   [7,11,14,16];
                   [8,12,15]};

    %描画パラメータ
    param.force_arrow_scale = 10^-9; %5*10^4
    param.magnetic_moment_arrow_scale = 10^-9;%100
    param.axis_norm = 0.5;


end

