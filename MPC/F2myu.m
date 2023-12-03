
% スラスター入力を磁気モーメントに変換。
u_myu = force2moment(s1, s0, num, u(1:end-1));

u_myu(end+1) = max(abs(u_myu));

r_norm = norm(r);
coilN = 100;
radius = 0.05;
I_max = 35;
mass = 1;
u_I = u_myu/(coilN * pi * radius^2);

disp("相手衛星から見た自分の相対位置r_val, 自分の磁気モーメントmyu1_val, 相手の磁気モーメントmyu2_valから導出される自分に働く磁力")
disp("バグ検証用出力")
r_val = s0(1:3) - s0(7:9);
myu1_val = u_myu(end-6:end-4);
myu2_val = u_myu(end-3:end-1);
F1 = f_func(r_val, myu1_val, myu2_val);
F2 = f_func(-r_val, myu2_val, myu1_val);
disp([F1;F2])

disp("最大電流")
disp(u_myu(end)/(coilN * pi * radius^2))
function u_myu = force2moment(s, s0, num, u)
    % 最終状態時以外の状態を抽出
    s = [s(1+6*num:end);s0];
    % 位置情報のみを抽出
    pos_vec = extract_3elements(s);
    % 相対位置を計算
    r = calc_rel_pos(pos_vec);
    u_myu = [];
    i = 1;
    n = length(r);
    while i <= n
        [myu1, myu2] = ru2myu(r(i:i+2), u(i:i+2));
        u_myu = [u_myu; myu1; myu2];
        i = i + 6;
    end
end

function [myu1, myu2] = ru2myu(r,u)
    r_norm = norm(r);
    coilN = 100;
    radius = 0.05;
    I_max = 35;
    mass = 1;
    myu01 = coilN * pi * radius^2 * I_max * r/r_norm;
    myu0 = 4*pi*1e-7; % 真空の透磁率
    
    D = calculateD(r, myu01);
    myu02 = 4*pi*r_norm^5/(3*myu0)*inv(D)*u*mass;
    myu1 = myu02;
    myu2 = myu01;
end


function D = calculateD(r, m1)
    x = r(1);
    y = r(2);
    z = r(3);
    r_norm = norm(r);
    D11 = 2*m1(1)*x + dot(m1, r) - 5*dot(m1, r)*x^2/r_norm^2;
    D12 = m1(2)*x + m1(1)*y - 5*dot(m1, r)*y*x/r_norm^2;
    D13 = m1(3)*x + m1(1)*z - 5*dot(m1, r)*z*x/r_norm^2;
    D21 = D12;
    D22 = 2*m1(2)*y + dot(m1, r) - 5*dot(m1, r)*y^2/r_norm^2;
    D23 = m1(3)*y + m1(2)*z - 5*dot(m1, r)*z*y/r_norm^2;
    D31 = D13;
    D32 = D23;
    D33 = 2*m1(3)*z + dot(m1, r) - 5*dot(m1, r)*z^2/r_norm^2;


    D = [D11,D21,D31;
         D12,D22,D32;
         D13,D23,D33;];

end


function rel_pos = calc_rel_pos(pos_vec)
    n = length(pos_vec); % ベクトルの長さを取得
    rel_pos = []; % 抽出された要素を格納するための空のベクトル
    i = 1;
    while i <= n
        pos_vec1 = pos_vec(i:i+2);
        pos_vec2 = pos_vec(i+3:i+5);
        rel_pios1 = pos_vec1 - pos_vec2;
        rel_pos2 = -rel_pios1;
        rel_pos = [rel_pos; rel_pios1; rel_pos2];
        i = i + 6;
    end
end


function extracted = extract_3elements(vector)
    n = length(vector); % ベクトルの長さを取得
    extracted = []; % 抽出された要素を格納するための空のベクトル

    i = 1;
    while i <= n
        % 現在の位置から3要素を抽出（範囲外にならないようにチェック）
        end_index = min(i+2, n);
        extracted = [extracted; vector(i:end_index)];
        
        % 6要素分進める（3要素抽出して3要素スキップ）
        i = i + 6;
    end
end

function F = f_func(r_val, myu1_val, myu2_val)
    syms xr yr zr myu11 myu12 myu13 myu21 myu22 myu23
    r = [xr; yr; zr];
    myu1 = [myu11; myu12; myu13];
    myu2 = [myu21; myu22; myu23];
    myu0 = 4*pi*1e-7; % 真空の透磁率

    f = 3*myu0/(4*pi)*(dot(myu1, myu2)/norm(r)^5 * r + dot(myu1, r)/norm(r)^5 * myu2 + dot(myu2, r)/norm(r)^5 * myu1 - 5*dot(myu1, r)*dot(myu2, r)/norm(r)^7*r);
    f_func0 = matlabFunction(f, 'vars', [xr, yr, zr, myu11, myu12, myu13, myu21, myu22, myu23]);
    xr_val = r_val(1);
    yr_val = r_val(2); 
    zr_val = r_val(3); 

    myu11_val = myu1_val(1);
    myu12_val = myu1_val(2);
    myu13_val = myu1_val(3);

    myu21_val = myu2_val(1);
    myu22_val = myu2_val(2);
    myu23_val = myu2_val(3);

    F = f_func0(xr_val, yr_val, zr_val, myu11_val, myu12_val, myu13_val, myu21_val, myu22_val, myu23_val);
end

