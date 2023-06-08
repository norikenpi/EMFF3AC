com_position = zeros(3,1);
com_velocity = zeros(3,1);

for i = 1:param.N
    com_position = com_position + satellites{i}.position;
    com_velocity = com_velocity + satellites{i}.velocity;
end

disp("com_position")
disp(com_position)
disp("com_velocity")
disp(com_velocity)