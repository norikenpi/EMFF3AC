%startManySimulationで大量のシミュレーションを同時に始めて、
%plotConvergePlotでプロットする
%Other/calculateStatesで初期条件の比較ができるよ

%clear
%result_C1 = zeros(1, 100);
%result_distance = zeros(1, 100);
%result_target_distance = zeros(1, 100);
%result_all_energy = zeros(1, 100);
%result_velocity = zeros(1, 100);
%result_freq_all = zeros(1, 100);
startTime = datetime;
param = setSimulationParameters();
number = 100;
seed0 = 1;

%{

% シミュレーションパラメータを設定
%result = [];
for i = 1:number
    
    type = 'C1';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_C1(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_C1 = result;
result_C101 = checkConvergeOrNot(result_C1);
makeResultFile(param, type, result_C1)


%result = [];
for i = 1:number
    type = 'distance';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_distance(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_distance = result;
result_distance01 = checkConvergeOrNot(result_distance);
makeResultFile(param, type, result_distance)

%result = [];
for i = 1:number
    type = 'target_distance';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_target_distance(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_target_distance = result;
result_target_distance01 = checkConvergeOrNot(result_target_distance);
makeResultFile(param, type, result_target_distance)

%result = [];
for i = 1:number
    type = "all_energy";
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_all_energy(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_all_energy = result;
result_all_energy01 = checkConvergeOrNot(result_all_energy);
makeResultFile(param, type, result_all_energy)
%}

%{
%result = [];
for i = 1:number
    type = "velocity";
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_velocity(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_velocity = result;
result_velocity01 = checkConvergeOrNot(result_velocity);
makeResultFile(param, type, result_velocity)
%}

%{
%result = [];
for i = 1:number
    type = 'freq_all';
    disp(i)
    disp(type)
    param.freq_all = true;
    param.j = seed0 + i - 1; %シード値
    simulateSatellite(param);
    result_freq_all(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_freq_all = result;
result_freq_all01 = checkConvergeOrNot(result_freq_all);
makeResultFile(param, type, result_freq_all)


%result = [];
for i = 1:number
    type = "separate_velocity";
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_separate_velocity(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_velocity = result;
result_separate_velocity01 = checkConvergeOrNot(result_separate_velocity);
makeResultFile(param, type, result_separate_velocity)
%}


for i = 1:number
    type = "Takahashi";
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result_Takahashi(param.j) = param.finished_time;
    %makeDataFile(param, i, type)
end
%result_velocity = result;
result_Takahashi01= checkConvergeOrNot(result_Takahashi);
makeResultFile(param, type, result_Takahashi)



endTime = datetime;
executionTime = endTime - startTime;
disp(['処理時間: ' char(executionTime)]);

mkdir(param.path_data)
filename_var = strcat(param.path_data, sprintf('/%s_var.mat', param.date));
save(filename_var);
filename_param = strcat(param.path_data, '/param.txt');
outputStructToTextFile(param, filename_param)
%result_all = result_freq_all01 + result_C101 + result_distance01 + result_target_distance01 + result_all_energy01 + result_separate_velocity01;
