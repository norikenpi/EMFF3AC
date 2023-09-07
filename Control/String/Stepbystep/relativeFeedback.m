function u = relativeFeedback(i, pair_satellite_idx, satellites, param, histories)

    
    %Hill方程式によって作られるオイラー近似離散状態方程式の係数
    A = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 2*param.n;
         0, -param.n^2, 0, 0, 0, 0;
         0, 0, 3*param.n^2, -2*param.n, 0, 0];
    
    A = [0, 0, 0, 1, 0, 0;
         0, 0, 0, 0, 1, 0;
         0, 0, 0, 0, 0, 1;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0, 0];
    m = satellites{i}.mass;
    
    %Hill方程式によって作られるオイラー近似離散状態方程式の係数
    B = [0, 0, 0;
         0, 0, 0;
         0, 0, 0;
         1/m, 0, 0;
         0, 1/m, 0;
         0, 0, 1/m];
    
    B_sharp = (B.'*B)\B.';
    
    %オイラー近似を利用した差分方程式の係数
    A_d = eye(6) + param.dt*A;
    
    %オイラー近似を利用した差分方程式の係数
    %B_d = param.dt*B;
    
    Q = param.Q;
    R = param.R;
    
    %リッカチ方程式を解いて最適ゲインを求める
    [~,K,~] = icare(A,B,Q,R,[],[],[]);
    
    %現在の相対位置速度と目標の相対位置速度
    relative_position = satellites{pair_satellite_idx}.position - satellites{i}.position;
    relative_velocity = satellites{pair_satellite_idx}.velocity - satellites{i}.velocity;
    
    relative_position_d = satellites{pair_satellite_idx}.position_d - satellites{i}.position_d;
    relative_velocity_d = satellites{pair_satellite_idx}.velocity_d - satellites{i}.velocity_d;
    relative_accelaration_d = [0;0;0];
    
    x = [relative_position; relative_velocity];
    xd = [relative_position_d; relative_velocity_d];
    vd = [relative_velocity_d; relative_accelaration_d];
    
    %フィードバック系を0に収束させるために入力変位を設定
    u_d = B_sharp*(vd - A_d*xd);
    
    %目標値と現在地のずれ
    x_tilda = x - xd;


    adjust_mat = [[10000,0,0,0,0,0];
                  [0,10000,0,0,0,0];
                  [0,0,10000,0,0,0];
                  [0,0,0,10000,0,0];
                  [0,0,0,0,10000,0];
                  [0,0,0,0,0,10000];];

    K = K * adjust_mat; 
    
    %x_tildaを0にするためのフィードバック
    u_tilda = -K*x_tilda;
    
    %元の状態方程式の入力(N) 自分から見た相手の衛星の制御力とは逆の制御力を自分に加える
    u = -(u_tilda + u_d);

