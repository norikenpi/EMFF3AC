clear


param = setSimulationParameters();
number = 1;
seed0 = 1;

% シミュレーションパラメータを設定
result = [];
for i = 1:number
    
    type = 'C1';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result(i) = param.finished_time;
    %makeDataFile(param, i, type)
end
result_C1 = result;
makeResultFile(param, type, result)

result = [];
for i = 1:number
    type = 'distance';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result(i) = param.finished_time;
    %makeDataFile(param, i, type)
end
result_distance = result;
makeResultFile(param, type, result)

result = [];
for i = 1:number
    type = 'target_distance';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result(i) = param.finished_time;
    %makeDataFile(param, i, type)
end
result_target_distance = result;
makeResultFile(param, type, result)

result = [];
for i = 1:number
    type = 'energy';
    disp(i)
    disp(type)
    param.j = seed0 + i - 1; %シード値
    param.pair_type = type;
    simulateSatellite(param);
    result(i) = param.finished_time;
    %makeDataFile(param, i, type)
end
result_energy = result;
makeResultFile(param, type, result)

result = [];
for i = 1:number
    type = 'freq_all';
    disp(i)
    disp(type)
    param.freq_all = true;
    param.j = seed0 + i - 1; %シード値
    simulateSatellite(param);
    result(i) = param.finished_time;
    %makeDataFile(param, i, type)
end
result_freq_all = result;
makeResultFile(param, type, result)



