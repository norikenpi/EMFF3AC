%startManySimulationで大量のシミュレーションを同時に始めて、
%showDataBar2で棒グラフ
%Other/calculateStatesで初期条件の比較ができるよ

%位置を固定すれば，平均速度がこれ以下にするのが必要条件であることがわかればいいよね．

%clear

%result_velocity = zeros(1, 100);
%initial_velocity_mean = zeros(1, 100);
startTime = datetime;
param = setSimulationParameters();
number = 10;
seed0 = 1;




%result = [];
for i = 1:number
    type = "velocity";
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_velocity(param.j) = param.finished_time;
    satellites = setInitialSatelliteStates(param);
    initial_velocity_mean(param.j) = calculateSatelliteVelocityMean(satellites);
    %makeDataFile(param, i, type)
end
%result_velocity = result;

result_velocity01 = checkConvergeOrNot(result_velocity);
makeResultFile(param, type, result_velocity)


mkdir(param.path_data)
filename_var = strcat(param.path_data, sprintf('/%s_var.mat', param.date));
save(filename_var);
filename_param = strcat(param.path_data, '/param.txt');
outputStructToTextFile(param, filename_param)
%result_all = result_freq_all01 + result_C101 + result_distance01 + result_target_distance01 + result_all_energy01 + result_separate_velocity01;