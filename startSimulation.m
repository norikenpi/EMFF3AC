


param = setSimulationParameters();
%param.t = 2000; 
%param.freq_all = false;
param.j = 1;
param.pair_type = "Takahashi";
param.pair_type = "velocity";
simulateSatellite(param);




