i1 = [1.5, 0, 0];
i2 = [1.5, 0, 0];
radius1 = 0.015;
radius2 = 0.015;
N1 = 17;
N2 = 17;
x0 = 0.15;
r = [x0, 0, 0];
F = calculateCurrent2Force(i1, i2, radius1, radius2, N1, N2, r, param);

%x1から無限遠まで積分
delta_x = 0.001;
x = x0;
E = 0;
for i = 1:500
    x = x + delta_x;
    r = [x, 0, 0];
    F = calculateCurrent2Force(i1, i2, radius1, radius2, N1, N2, r, param);
    E = E + F * delta_x;
end
disp(E)
