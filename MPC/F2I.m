vec = DistanceFourthPower(s1, s0, num);
%scale = 10^(5);
u_I = u(1:end-1).*vec * 10^(6);
u_I(end+1) = max(abs(u_I));
disp("最大入力 u_max")
disp(u_I(3*num*N+1) * 10^(-6))
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
%disp(s12)
%disp(sl2)

ss = P * u_I + Q * s0 + R;

%disp(ss)