% シミュレーションパラメータを設定する関数。サンプリング間隔、シミュレーション時間、比例ゲイン、微分ゲイン、最大力などのパラメータを設定します。
function param = setSimulationParameters()
    myu = 3.986*10^14;
    earth_radius = 6378.140*10^3; % 地球半径
    altitude = 500*10^3; % 高度
    r_star = earth_radius + altitude; % 地心からの距離
    param.n = sqrt(myu/r_star^3); % 地球を周回する衛星の角速度


    param.dt = 0.1; % サンプリング間隔
    param.t = 1000; % シミュレーション時間
    param.Kp = 10^(-6); % 比例ゲイン10^(-6)
    param.Kd = 0.05; % 微分ゲイン

    param.C1_min = 10^-7; % 最小ドリフト
    param.j = 9; %シード値
    param.C1_ini = 0.01;%初期C1の最大値
    param.safety_distance = 0.05; % 衝突回避制御を実施するための距離
    param.avoid_collision_magnetic_moment = 0.00005; % 衝突回避制御を行うときの磁気ダイポールゲイン
    param.max_distance = 2; % 通信可能距離
    param.min_distance_nopair = 0.05; % ペアリングした衛星よりも近くに別の衛星があるとき、この距離以内にある場合、制御しない。
    

    param.N = 8; % 衛星の数

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
                       [13,15]};
    
    %{
    param.timetable = {[1,2];
                       [3,4];
                       [1,3];
                       [2,4]};
   %}


    

    %描画パラメータ
    param.force_arrow_scale = 10^-9; %5*10^4
    param.magnetic_moment_arrow_scale = 10^-9;%100
    param.axis_norm = 1;


end