%{

A=randn(100,30);
b=randn(100,1);

cvx_begin
variable x(30)
x>=0
sum(x)==1
minimize(norm(A*x-b))
cvx_end

x
%}

A = rand(3,3);
B = rand(3,3);
alpha = 1e-2;
n = 3;
I = eye(n);
cvx_begin sdp quiet
variable traceM nonnegative
variable alpha_max nonnegative
variable Wy(n,n) semidefinite
variable J(n,n) symmetric semidefinite %nonnegative
variable epsilon nonnegative
variable underw nonnegative
variable overw nonnegative
variable chi nonnegative
minimize 1e1*alpha_max
subject to
    I <= Wy <= chi*I; %<= overw*I;
    [A*Wy+alpha*Wy + (A*Wy+alpha*Wy).'-2*B*(B.')*traceM, I;
        I, -alpha_max*I] <= 0;
cvx_end