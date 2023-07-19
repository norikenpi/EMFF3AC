%startManySimulationで大量のシミュレーションを同時に始めて、
%plotConvergePlotでプロットする

%clear
result_C1 = zeros(1, 100);
result_distance = zeros(1, 100);
result_target_distance = zeros(1, 100);
result_all_energy = zeros(1, 100);
result_velocity = zeros(1, 100);
result_freq_all = zeros(1, 100);
startTime = datetime;
param = setSimulationParameters();
number = 4;
seed0 = 1;

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
makeResultFile(param, type, result_all_energy)

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
makeResultFile(param, type, result_velocity)

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
makeResultFile(param, type, result_freq_all)


endTime = datetime;
executionTime = endTime - startTime;
disp(['処理時間: ' char(executionTime)]);