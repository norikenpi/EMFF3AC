% 完全なfar-field

syms xr yr zr myu11 myu12 myu13 myu21 myu22 myu23

r = [xr; yr; zr];
myu1 = [myu11; myu12; myu13];
myu2 = [myu21; myu22; myu23];

f = dot(myu1, myu2)/norm(r)^5 * r + dot(myu1, r)/norm(r)^5 * myu2 + dot(myu2, r)/norm(r)^5 * myu1 - 5*dot(myu1, r)*dot(myu2, r)/norm(r)^7*r;

df_dxr = diff(f, xr);
df_dyr = diff(f, yr);
df_dzr = diff(f, zr);
df_dmyu11 = diff(f, myu11);
df_dmyu12 = diff(f, myu12);
df_dmyu13 = diff(f, myu13);
df_dmyu21 = diff(f, myu21);
df_dmyu22 = diff(f, myu22);
df_dmyu23 = diff(f, myu23);



disp('偏微分 df/dxr:');
disp(df_dxr);
disp('偏微分 df/dyr:');
disp(df_dyr);

df_dxr_func = matlabFunction(df_dxr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dyr_func = matlabFunction(df_dyr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dzr_func = matlabFunction(df_dzr, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu11_func = matlabFunction(df_dmyu11, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu12_func = matlabFunction(df_dmyu12, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu13_func = matlabFunction(df_dmyu13, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu21_func = matlabFunction(df_dmyu21, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu22_func = matlabFunction(df_dmyu22, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
df_dmyu23_func = matlabFunction(df_dmyu23, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
f_func = matlabFunction(f, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);

xr_val = 1;
yr_val = 1.1;
zr_val = 1.2;
myu1x_val = 1.3;
myu1y_val = 1.4;
myu1z_val = 1.5;
myu2x_val = 1.3;
myu2y_val = 1.4;
myu2z_val = 1.5;

df_dxr_at_point = subs(df_dxr, [xr yr zr myu11 myu12 myu13 myu21 myu22 myu23], [xr_val yr_val zr_val myu1x_val myu1y_val myu1z_val myu2x_val myu2y_val myu2z_val]);
df_dyr_at_point = subs(df_dyr, [xr yr zr myu11 myu12 myu13 myu21 myu22 myu23], [xr_val yr_val zr_val myu1x_val myu1y_val myu1z_val myu2x_val myu2y_val myu2z_val]);
df_dxr_at_point_decimal = double(df_dxr_at_point);
df_dyr_at_point_decimal = double(df_dyr_at_point);

disp("matlab sym")
disp(['偏微分 df/dx at (', num2str(xr_val), ',', num2str(yr_val), '):']);
disp(df_dxr_at_point_decimal);
disp(['偏微分 df/dy at (', num2str(xr_val), ',', num2str(yr_val), '):']);
disp(df_dyr_at_point_decimal);

disp("関数化")
disp(df_dxr_func(xr_val, yr_val, zr_val, myu1x_val, myu1y_val, myu1z_val, myu2x_val, myu2y_val, myu2z_val))
disp(df_dyr_func(xr_val, yr_val, zr_val, myu1x_val, myu1y_val, myu1z_val, myu2x_val, myu2y_val, myu2z_val))

disp("手計算")
r_val = [xr_val; yr_val; zr_val];
myu1_val = [myu1x_val; myu1y_val; myu1z_val];
myu2_val = [myu2x_val; myu2y_val; myu2z_val];


df_dxr_at_point_hand1 = -5*xr_val*dot(myu1_val, myu2_val)*norm(r_val)^(-7)*r_val + dot(myu1_val, myu2_val)*norm(r_val)^(-5)*[1;0;0];
df_dxr_at_point_hand2 = -5*xr_val*dot(myu1_val, r_val)*norm(r_val)^(-7)*myu2_val + myu1x_val*norm(r_val)^(-5)*myu2_val;
df_dxr_at_point_hand3 = -5*xr_val*dot(myu2_val, r_val)*norm(r_val)^(-7)*myu1_val + myu2x_val*norm(r_val)^(-5)*myu1_val;
df_dxr_at_point_hand4 = 35*xr_val*dot(myu1_val, r_val)*dot(myu2_val, r_val)*norm(r_val)^(-9)*r_val - 5*myu1x_val*dot(myu2_val, r_val)*norm(r_val)^(-7)*r_val...
                       -5*myu2x_val*dot(myu1_val, r_val)*norm(r_val)^(-7)*r_val - 5*dot(myu1_val, r_val)*dot(myu2_val, r_val)*norm(r_val)^(-7)*[1;0;0];

df_dxr_at_point_hand = df_dxr_at_point_hand1 + df_dxr_at_point_hand2 + df_dxr_at_point_hand3 + df_dxr_at_point_hand4;
disp(df_dxr_at_point_hand)


disp("dfdr")
func_cell = {df_dxr_func, df_dyr_func, df_dzr_func, df_dmyu11_func, df_dmyu12_func, df_dmyu13_func, df_dmyu21_func, df_dmyu22_func, df_dmyu23_func};

disp("dfdr")
disp(dfdr_func(r_val, myu1_val, myu2_val, func_cell))
disp("dfdmyu1")
disp(dfdmyu1_func(r_val, myu1_val, myu2_val, func_cell))
disp("dfdmyu2")
disp(dfdmyu2_func(r_val, myu1_val, myu2_val, func_cell))

disp("相手衛星から見た自分の相対位置r_val, 自分の磁気モーメントmyu1_val, 相手の磁気モーメントmyu2_valから導出される自分に働く磁力")
F = f_func2(r_val, myu1_val, myu2_val, f_func);
disp(F)



function dfdr = dfdr_func(r_val, myu1_val, myu2_val, func_cell)
    df_dxr_func = func_cell{1};
    df_dyr_func = func_cell{2};
    df_dzr_func = func_cell{3};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dxr = df_dxr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dyr = df_dyr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dzr = df_dzr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    dfdr = [df_dxr, df_dyr, df_dzr]; % 3×3
end

function dfdmyu1 = dfdmyu1_func(r_val, myu1_val, myu2_val, func_cell)
    df_dmyu11_func = func_cell{4};
    df_dmyu12_func = func_cell{5};
    df_dmyu13_func = func_cell{6};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dmyu1x = df_dmyu11_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu1y = df_dmyu12_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu1z = df_dmyu13_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);

    dfdmyu1 = [df_dmyu1x, df_dmyu1y, df_dmyu1z]; % 3×3
end

function dfdmyu2 = dfdmyu2_func(r_val, myu1_val, myu2_val, func_cell)
    df_dmyu21_func = func_cell{7};
    df_dmyu22_func = func_cell{8};
    df_dmyu23_func = func_cell{9};

    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    df_dmyu2x = df_dmyu21_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu2y = df_dmyu22_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
    df_dmyu2z = df_dmyu23_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);

    dfdmyu2 = [df_dmyu2x, df_dmyu2y, df_dmyu2z]; % 3×3    
end

function F = f_func2(r_val, myu1_val, myu2_val, f_func)
    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    F = f_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
end

%{
syms x y
f = x^2 + y^2 + x*y;
df_dx = diff(f, x);
df_dy = diff(f, y);
disp('偏微分 df/dx:');
disp(df_dx);
disp('偏微分 df/dy:');
disp(df_dy);
%}