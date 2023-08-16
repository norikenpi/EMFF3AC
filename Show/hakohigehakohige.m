result_target_distance_plus = result_target_distance(result_target_distance > 0);
result_distance_plus = result_distance(result_distance > 0);

result_velocity_plus = result_velocity(result_velocity > 0);
result_all_energy_plus = result_all_energy(result_all_energy > 0);
result_C1_plus = result_C1(result_C1 > 0);
result_Takahashi_plus = result_Takahashi(result_Takahashi > 0);



%result_velocity5105_plus = result_velocity5105;
%result_velocity3105_plus = result_velocity3105;
%result_velocity104_plus = result_velocity104;￥


%result_velocity5104_plus = result_velocity5104;
%result_velocity5104_plus = 0;
%result_velocity5105_minus = result_velocity5105(result_velocity5105 < 0);
%result_velocity3105_minus = result_velocity3105(result_velocity3105 < 0);
%result_velocity104_minus = result_velocity104( result_velocity104 < 0);
%result_velocity5104_minus = result_velocity5104(result_velocity5104 < 0);


results_data = {};
labels = {'目標相対位置誤差最大', '衛星間距離最小','相対速度最大', '相対エネルギー最大', '分離度最大', 'T out最小'};
%展開成功したときの展開完了するまでの時間を箱ひげ図で図示．

result_distance_plus = 0;
showResultHakohige(result_target_distance_plus, result_distance_plus, result_velocity_plus, result_all_energy_plus, result_C1_plus, result_Takahashi_plus, labels);
result_distance_plus = [];
%展開失敗数を棒グラフで図示
showBarplot(result_target_distance_plus, result_distance_plus, result_velocity_plus, result_all_energy_plus, result_C1_plus, result_Takahashi_plus, labels)