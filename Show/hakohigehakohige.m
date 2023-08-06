
result_velocity5105_plus = result_velocity5105(result_velocity5105 > 0);
result_velocity104_plus = result_velocity104( result_velocity104 > 0);
result_velocity5104_plus = result_velocity5104(result_velocity5104 > 0);

result_velocity5105_plus = result_velocity5105;
result_velocity104_plus = result_velocity104;
result_velocity5104_plus = result_velocity5104;


result_velocity5105_minus = result_velocity5105(result_velocity5105 < 0);
result_velocity104_minus = result_velocity104( result_velocity104 < 0);
result_velocity5104_minus = result_velocity5104(result_velocity5104 < 0);


results_data = {};
labels = {'5×10^(-5)','10^(-4)', '5×10^(-4)'};
showResultHakohige(result_velocity5105_plus, result_velocity104_plus, result_velocity5104_plus, labels);

showBarplot(result_velocity5105_minus, result_velocity104_minus, result_velocity5104_minus)