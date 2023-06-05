function [a_, T] = baseTransformaion(a, b)
    %あるベクトルaに直交する基底を求めたい。また、その基底はベクトルbが[b1, b2, 0]という形になるようにする。
    %その基底におけるaを出力する。
    e1 = a;
    e2 = cross(a, b);
    e3 = cross(e1, e2);
    T = [e1,e2,e3];
    a_ = T \ a;
end