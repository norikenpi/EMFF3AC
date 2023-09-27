function max_eigenvalue = evaluateFormation(N, DD_)
    %衛星間距離
    %d = 1;
    
    %一定の領域内の誤差を仮定
    %delta = 0.01;
    %delta_dot = 0.01;
    
    %定数
    
    
    %最大電流
    %Imax = 1;
    
    %ゲイン
    %k = C * Imax^2/(d^4 * delta);
    
    
    DD = generateMatrix(N);
    
    D = kron(DD, eye(3));
    
    %フォーメーションのネットワークを定義
    %{
    D_1 = [-2, -1;
         1, -1;
         1, 2] * k/4;
    
    D_2 = [-1, 0;
         1, -1;
          0, 1]*k/2;
    %}
    
    %D_ = D_2;
    
    D_ = kron(DD_, eye(3));
    
    kD_ = D_;
    kLe = D.' * kD_;
    
    Acl = [zeros(6), eye(6); -kLe, -kLe];
    
    
    eigenvalues = eig(Acl); % eig() 関数で固有値を計算
    max_eigenvalue = max(eigenvalues);
    %disp(max_eigenvalue);

