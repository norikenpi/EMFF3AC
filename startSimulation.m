
%warning('off','all')
%これで警告をoff

param = setSimulationParameters();
%param.t = 2000; 
%param.freq_all = false;
param.j = 7;
param.pair_type = "Takahashi";
param.pair_type = "freq_all";
simulateSatellite(param);




