% far-fieldの第1項だけを考慮。手計算の偏微分と合っているかを確認
%{
syms xr yr zr myu11 myu12 myu13 myu21 myu22 myu23

r = [xr; yr; zr];
myu1 = [myu11; myu12; myu13];
myu2 = [myu21; myu22; myu23];

f = dot(myu1, myu2)/norm(r)^5 * r;
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

df_dxr_func = matlabFunction(df_dxr);
df_dyr_func = matlabFunction(df_dyr);
df_dzr_func = matlabFunction(df_dzr);
df_dmyu11_func = matlabFunction(df_dmyu11);
df_dmyu12_func = matlabFunction(df_dmyu12);
df_dmyu13_func = matlabFunction(df_dmyu13);
df_dmyu21_func = matlabFunction(df_dmyu21);
df_dmyu22_func = matlabFunction(df_dmyu22);
df_dmyu23_func = matlabFunction(df_dmyu23);


xr_val = 1;
yr_val = 1;
zr_val = 1;
myu11_val = 1;
myu12_val = 1;
myu13_val = 1;
myu21_val = 1;
myu22_val = 1;
myu23_val = 1;

df_dxr_at_point = subs(df_dxr, [xr yr zr myu11 myu12 myu13 myu21 myu22 myu23], [xr_val yr_val zr_val myu11_val myu12_val myu13_val myu21_val myu22_val myu23_val]);
df_dyr_at_point = subs(df_dyr, [xr yr zr myu11 myu12 myu13 myu21 myu22 myu23], [xr_val yr_val zr_val myu11_val myu12_val myu13_val myu21_val myu22_val myu23_val]);
df_dxr_at_point_decimal = double(df_dxr_at_point);
df_dyr_at_point_decimal = double(df_dyr_at_point);

disp("matlab sym")
disp(['偏微分 df/dx at (', num2str(xr_val), ',', num2str(yr_val), '):']);
disp(df_dxr_at_point_decimal);
disp(['偏微分 df/dy at (', num2str(xr_val), ',', num2str(yr_val), '):']);
disp(df_dyr_at_point_decimal);

disp("関数化")
disp(df_dxr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val))
disp(df_dyr_func(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val))

disp("手計算")
r_val = [xr_val; yr_val; zr_val];
myu1_val = [myu11_val; myu12_val; myu13_val];
myu2_val = [myu21_val; myu22_val; myu23_val];

df_dxr_at_point_hand = -5*xr_val*dot(myu1_val, myu2_val)*norm(r_val)^(-7)*r_val + dot(myu1_val, myu2_val)*norm(r_val)^(-5)*[1;0;0];
disp(df_dxr_at_point_hand)


disp("dfdr")
func_cell = {df_dxr_func, df_dyr_func, df_dzr_func};

disp(dfdr_func(r_val, myu1_val, myu2_val, func_cell))




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
%}

syms x y
f = x^2 + y + x*y;
df_dx = diff(f, x);
df_dy = diff(f, y);
disp('偏微分 df/dx:');
disp(df_dx);
disp('偏微分 df/dy:');
disp(df_dy);
df_dy_func = matlabFunction(df_dy);
x_val = 0;
y_val = 0;
disp(df_dy_func(x_val))

