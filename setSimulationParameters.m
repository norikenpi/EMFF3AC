% シミュレーションパラメータを設定する関数。サンプリング間隔、シミュレーション時間、比例ゲイン、微分ゲイン、最大力などのパラメータを設定します。
function param = setSimulationParameters()
    param.myu = 3.986*10^14; %重力定数
    param.earth_radius = 6378.140*10^3; % 地球半径
    param.altitude = 500*10^3; % 高度
    r_star = param.earth_radius + param.altitude; % 地心からの距離
    param.n = sqrt(param.myu/r_star^3); % 地球を周回する衛星の角速度

    param.approximation = 'trapezoid'; % trapezoid or euler
    param.magnetic_model = 'far_field'; % near_field or far_field 発生させた電流に対して，発生する磁力をどのモデルで計算するか
    param.control_magnetic_model = 'far_field'; % near_field or far_field 所望の力に対して発生させる磁力を計算するときにどのもでるを使うか
    param.hill_on = true; % 深宇宙想定か、軌道上想定か
    param.start_state = 0;
    param.start_state = [4,1,1];
    param.current_type = 'AC'; % DC or AC
    param.dt = 0.1; % シミュレーションタイムステップ
    param.time_step = 0.1; % 制御タイムステップ

    
    param.t = 3000; % シミュレーション時間 
    %param.t = round(2*pi/param.n);
    param.initial_error = 0.005;%初期誤差
    param.satellite_initial_distance = 0.15; %初期衛星間距離
    param.satellite_desired_distance = 0.30; %衛星間距離

    %near_field
    param.coil_split = 10;

    %保存先
    param.path = 'C:/Users/masam/lab/30_simscape/20_磁石/';
    %param.path = 'C:/Users/nakan/OneDrive/デスクトップ/kubota/';
    param.date = datetime('now','Format', 'yyyy-MM-dd-HH-mm-ss');
    param.path_data = sprintf(strcat(param.path, 'movie/%s'), param.date);

    param.pair_time = 0.1; % ペアリングされてる時間

    param.N = 8; % 衛星の数    

    %Danil式パラメータ
    param.Kp = 10; % 比例ゲイン10^(-6)
    param.Kd = 0.05; % 微分ゲイン

    param.Kp_avoid = 100000000; % 比例ゲイン10^(-6)
    param.C1_min = 10^-7; % 最小ドリフト
    param.j = 3; %シード値
    param.C1_ini = 0.1;%初期C1の最大値
    param.safety_distance = 0.2; % 衝突回避制御を実施するための距離
    param.avoid_collision_magnetic_moment = 0.00005; % 衝突回避制御を行うときの磁気ダイポールゲイン
    param.max_distance = 1; % 通信可能距離
    param.min_distance_nopair = 0.05; % ペアリングした衛星よりも近くに別の衛星があるとき、この距離以内にある場合、制御しない。


    param.control_border = 0.01; %常に目標相対位置誤差に基づく制御
    
    %衛星の数-1の周波数を使う場合true
    param.freq_all = false;

    %param.pair_type = 'C1';
    %param.pair_type = 'distance';%このパラメータを変えることで、何を元にペアを組むかを決める。
    param.pair_type = 'velocity';
    %param.pair_type = 'target_distance';
    %param.pair_type = 'target_distance';

    %制御の終了条件
    param.border = 0.02;
    param.diverge_border = 1;
    param.finished_time = param.t;

    %衛星の初期パラメータ
    param.angular_velocity = [0; 0; 0.1]; % 角速度
    param.magnetic_moment = [0; 0.01; 0]; % 磁気モーメント

    param.mean_pos = [0, 0, 0]; %位置平均
    param.std_pos = [0.05, 0.05, 0.05]; %位置標準偏差[0.05, 0.05, 0.05]
    param.mean_vel = [0, 0, 0]; %速度平均
    param.std_vel = [0, 0, 0]; %速度標準偏差[5*10^-5, 5*10^-5, 5*10^-5]
    %param.std_vel = [10^-4, 10^-4, 10^-4]; %速度標準偏差
    %param.std_vel = [15*10^-5, 15*10^-5, 15*10^-5]; %速度標準偏差


    
    param.mass = 1; % 衛星質量
    param.moment_of_inertia = 1; % 慣性モーメント

    %2018年の野田さんの宇科連準拠
    param.coilN = 17; % 巻き数
    param.I_max = 10; % 最大電流
    param.radius = 0.05; %衛星半径
    param.max_magnetic_moment = param.coilN * param.I_max * pi * param.radius^2; % 最大磁気モーメント
   

    %紐ありの場合
    param.cof = 0.8; %反発係数
    param.length = 0.17;

    %最適制御パラメータ
    param.Q = diag([1, 1, 1, 10^8, 10^8, 10^8]); %diag([1, 1, 1, 10^4, 10^4, 10^4])
    param.R = diag([1, 1, 1]);



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
    

    

    param.timetable = [[[1,9],[4,12]];
                       [[2,10],[0,0]];
                       [[3,11],[0,0]];
                       [[9,13],[12,16]];
                       [[10,14],[0,0]];
                       [[11,15],[0,0]];
                       [[5,13],[8,16]];
                       [[6,14],[0,0]];
                       [[7,15],[0,0]];
                       [[1,2],[0,0]];
                       [[2,3],[0,0]];
                       [[3,4],[0,0]];
                       [[9,10],[0,0]];
                       [[10,11],[0,0]];
                       [[11,12],[0,0]];
                       [[13,14],[0,0]];
                       [[14,15],[0,0]];
                       [[15,16],[0,0]];
                       [[5,6],[0,0]];
                       [[6,7],[0,0]];
                       [[7,8],[0,0]]];

    %4衛星用
    %{
    param.timetable = [[[1,2],[0,0]];
                       [[2,4],[0,0]];
                       [[3,4],[0,0]];
                       [[1,3],[0,0]]];
    %}
    


    

    % 紐を使ったときに使うパラメータ
    % 各衛星と接続している衛星
    %{
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

        param.set =  {[2,3,4];
                      [1,3,4];
                      [1,2,4];
                      [1,2,3];
                   };
        param.set =  [1;2;3;4;5];
    %}

    %4衛星用
    %{
    param.set = [[1,2];
                 [2,3];
                 [3,4];
                 [1,4]];
    %}
    param.set_AC = [[1,2];
                 [2,3];
                 [3,4];
                 [9,10];
                 [10,11];
                 [11,12];
                 [13,14];
                 [14,15];
                 [15,16];
                 [5,6];
                 [6,7];
                 [7,8];
                 [1,9];
                 [2,10];
                 [3,11];
                 [4,12];
                 [9,13];
                 [10,14];
                 [11,15];
                 [12,16];
                 [13,5];
                 [14,6];
                 [15,7];
                 [16,8]];



    %交流
    %4衛星用
    %{
    param.frequency_set = [1;
                           2;
                           3;
                           4];
    %}
    param.frequency_set = [7;
                           6;
                           5;
                           12;
                           11;
                           10;
                           17;
                           16;
                           15;
                           4;
                           21;
                           20;
                           1;
                           9;
                           8;
                           1;
                           2;
                           14;
                           13;
                           2;
                           3;
                           19;
                           18;
                           3];
    
    param.frequency_set = [1;
                           2;
                           3;
                           4;
                           5;
                           6];
    nCn_1 = (param.N*(param.N-1))/2;
    param.frequency_set = (1:nCn_1).';

    %4衛星用
    param.frequency = [10;
                       20;
                       30;
                       40;
                       50;
                       60;
                       70;
                       80;
                       90;
                       100;
                       110;
                       120;
                       130;
                       140;
                       150;
                       160;
                       170;
                       180;
                       190;
                       200;
                       210]*pi;

    param.frequency = [10;
                       20;
                       30;
                       40;
                       50;
                       60]*pi; %rad/s  

    param.frequency = (10:10:nCn_1*10).' * 2 * pi;
     
  
    



    %描画パラメータ
    param.force_arrow_scale = 5*10^2; %5*10^4
    
    param.magnetic_moment_arrow_scale = 10^-9;%100
    param.velocity_arrow_scale = 0;%100
    param.axis_norm = 1; %0.5
    param.pause = 172;

    param.force_arrow_scale = 0; %5*10^4
    param.magnetic_moment_arrow_scale = 5*10^4; %5*10^4
    param.velocity_arrow_scale = 5*10; %5*10^4


end

