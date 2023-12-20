 relative_mat = [eye(6),-eye(6)];
s = P * u_list2 + Q * s0;
for i = 1:N
    a(i) = norm(s(6*(i-1)+1:6*i+3));

end