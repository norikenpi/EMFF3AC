a = [1;1;1];
b = [1;-2;1];

[a_, T] = baseTransformaion(a, b);

disp(a_)
disp(T*a_)
