% 相対距離の4乗を格納したベクトルを作成。これをスラスター入力にかけることで、同じ入力を距離の4乗に反比例した入力で再現した場合の入力が求まる。
vec = RelativeDistanceFourthPower(s1, s0, num);

u_I = u(1:end-1).*vec * 10^(6);
u_I(end+1) = max(abs(u_I));
s11 = A_d*s0 + B_d*u_I(end-3:end-1)/norm(s0(1:3))^4;
s12 = A_d*s11 + B_d*u_I(end-6:end-4)/norm(s11(1:3))^4;


A11 = A_list{1};
B11 = B_list{1};
C11 = C_list{1};
A12 = A_list{2};
B12 = B_list{2};
C12 = C_list{2};


sl1 = A11*s0 + B11*u_I(end-3:end-1) + C11;
sl2 = A12*s11 + B12*u_I(end-6:end-4) + C12;
disp(s12)
disp(sl2)

ss = P * u_I + Q * s0 + R;
disp(ss)