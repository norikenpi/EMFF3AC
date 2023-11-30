
%衛星2から衛星1を見た相対位置ベクトル
r = [0;1;0];

%衛星1に加えたい力
u1 = [-3.18;-3.18;-3.18]*10^-7;

r_norm = norm(r);
coilN = 100;
radius = 0.05;
I_max = 20;
mass = 1;

myu1 = coilN * pi * radius^2 * I_max * r/r_norm;
myu0 = 4*pi*1e-7; % 真空の透磁率

D = calculateD(r, myu1);
myu2 = 4*pi*r_norm^5/(3*myu0)*inv(D)*u1*mass;

f = f_func(r, myu1, myu2, myu0);
disp("I1")
disp(myu1/(coilN * pi * radius^2))
disp("I2")
disp(myu2/(coilN * pi * radius^2))
disp("f")
disp(f)


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

function f = f_func(r, myu1, myu2, myu0)
    f = 3*myu0/(4*pi) * (dot(myu1, myu2)/norm(r)^5 * r + dot(myu1, r)/norm(r)^5 * myu2 + dot(myu2, r)/norm(r)^5 * myu1 - 5*dot(myu1, r)*dot(myu2, r)/norm(r)^7*r);
end
