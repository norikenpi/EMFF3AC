%satteliteというcell 配列を入れてそこに位置を記録していこう。
%高橋座標系になっていることに注意。

num = 1;
dt = 1;
N = 10000;
n = 0.0011; % 0.0011
m = 1; % 1

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     3*n^2, 0, 0, 0, 2*n, 0;
     0, 0, 0, -2*n, 0, 0;
     0, 0, -n^2, 0, 0, 0];

B = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     1/m, 0, 0;
     0, 1/m, 0;
     0, 0, 1/m]; % 6×3

% 2衛星に関する離散時間状態方程式の係数行列
A_d = eye(6)+ dt*A; % 6num×6num
B_d = dt*B; % 6num×3num



d_target = 0.925;

rr1 = d_target/(2*sqrt(3));
if num == 2
    rr1 = d_target/4;
end
rr2 = sqrt(2)*d_target/2;

rr = [rr1,rr2];
rr1 = rr(1);
%s0 = [-2*rr1*cos(n*N*dt); sqrt(3)*rr1*sin(n*N*dt); rr1*sin(n*N*dt); 2*n*rr1*sin(n*N*dt); sqrt(3)*n*rr1*cos(n*N*dt); n*rr1*cos(n*N*dt)];
%s0 = [rr1*sin(0); 2*rr1*cos(0); sqrt(3)*rr1*sin(0); n*rr1*cos(0); -2*n*rr1*sin(0); sqrt(3)*n*rr1*cos(0)];
s0 = [0.8146;0.2238;0.0266;1.231027657282976e-04;-0.0018;2.132202447936586e-04];
s = zeros(N,6);
s(1,:) = s0.';

thetaP = pi/6;
rd = 0;

for i = 1:N
    X = s(i,:).';
    C110 = coord2const(X, n);
    kA = 2e-3;%Gains(1);
    kB = 1e-3;
    C1 = C110(1); C4 = C110(4); C5 = C110(5);
    C2 = C110(2); C3 = C110(3);
    r_xy = C110(7); phi_xy = C110(8); %phi_xy = atan2(o_r_ji(1),o_r_ji(2)/2);
    
    %r_xyd = rd*sin(abs(thetaP)); %目標値
    %r_zd = rd*cos(abs(thetaP)); %目標値
    C4d = 3*n*C1/kA; %目標値
    dC4 = C4-C4d; %C4偏差
    
    %r_zd = r_xy/tan(thetaP);
    %r_zd = sqrt(C2^2+C3^2)/tan(thetaP);
    %C5d = r_zd*cos(phi_xy);
    %C5d = r_zd*cos(atan2(C3,C2));
    %C5d = r_zd*C2/sqrt(C2^2+C3^2);
    C5d = C2/tan(thetaP);
    u_A = kA*n*[1/2*dC4;-C1];  
    u = [u_A;-kB*n*(C5-C5d)];
    u = [0;0;0];
    disp(u)
    s(i+1,:) = (A_d * s(i,:).' + B_d * u).';
    disp(s(i+1,:))
end

satellites{1} = s(:,1:3);
plot_s(satellites, num, N, rr, d_target)
function C = coord2const(X, w)
    %% HCW constants calculated from the free motion equation
    C(1) = 2*X(1)+X(5)/w;
    C(2) = X(4)/w;
    C(3) = -3*X(1)-2*X(5)/w;
    C(4) = X(2)-2*X(4)/w;
    C(5) = X(6)/w;
    C(6) = X(3);
    %%
    r_xy = sqrt(C(2)^2+C(3)^2);
    theta_xy = atan2(C(3),C(2));
    r_z = sqrt(C(5)^2+C(6)^2);
    theta_z = atan2(C(6),C(5));
    C(7) = r_xy;
    C(8) = theta_xy;
    C(9) = r_z;
    C(10) = theta_z;
    %%
    %{
    if rs^2-(2*C(1))^2 > 0 && abs(C(1)) > 0
        Tconn = sign(C(1))*C(4)+sqrt(rs^2-(2*C(1))^2)/(3*w*abs(C(1)));
    else
        Tconn = 0;
    end
    C(11) = Tconn;
    %}
end

function plot_s(satellites, num, N, rr, d_target)
    % 2衛星の動画を表示。
    %3次元座標

    % ビデオライターオブジェクトの作成
    v = VideoWriter('points_motion_3D.avi'); % AVIファイル形式で動画を保存
    % 画質の設定（例：品質を最大に）
    v.Quality = 100;
    open(v);
    
    % フィギュアの作成
    figure;
    axis equal;
    xlim([-d_target*1.5, d_target*1.5]/3); % x軸の範囲を調整
    ylim([-d_target*1.5, d_target*1.5]/3); % y軸の範囲を調整
    zlim([-d_target*1.5, d_target*1.5]/3); % z軸の範囲を調整
    hold on;
    grid on; % グリッドを表示
    
    set(gca, 'ZDir', 'reverse')
    
    % 軸のラベルを設定
    xlabel('X[m](地心方向)');
    ylabel('Y[m](軌道進行方向)');
    zlabel('Z[m](軌道面垂直方向)');
    
    theta = linspace(0, 2 * pi, 100); % 0から2πまでの角度を生
    colors = hsv(num); % HSVカラースペースを使用してN個の異なる色を生成
    
    % 各フレームでの点の位置をプロットし、そのフレームを動画に書き込む
    for i = 1:100:N
        cla;
        
        for j = 1:length(rr)
            % レコード盤軌道をプロット
            %x1 = -2*rr(j)*cos(theta); % x座標を計算
            %y1 = sqrt(3)*rr(j)*sin(theta); % y座標を計算
            %z1 = rr(j)*sin(theta);
            x1 = rr(j)*sin(theta);% x座標を計算
            y1 = 2*rr(j)*cos(theta); % y座標を計算2*rr(j)*cos(theta);
            z1 = sqrt(3)*rr(j)*sin(theta);
            plot3(x1, y1, z1, 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1); % 円を灰色で描画
        end

        % x軸方向の点線を描画
        y_position = -1:0.1:1; % x軸方向の点線のx座標を指定
        x_position = zeros(size(y_position)); % y座標はすべて0に設定
        plot(x_position, y_position, 'k--', 'LineWidth', 1.5); % 点線を描画
       
        for j = 1:num
            % 軌道をプロット
            plot3(satellites{j}(1:N,1), satellites{j}(1:N,2), satellites{j}(1:N,3), '-', 'Color', colors(j,:));
            % 衛星をプロット
            plot3(satellites{j}(i,1), satellites{j}(i,2), satellites{j}(i,3), '.', 'MarkerSize', 60, 'Color', colors(j,:));
            % 衛星の初期値をプロット
            plot3(satellites{j}(1,1), satellites{j}(1,2), satellites{j}(1,3), 'o', 'MarkerSize', 5, 'Color', colors(j,:));
        end
        % 視点を変更
        azimuth = 225; % 方位角
        elevation = 30; % 仰角
        view(azimuth, elevation);
    
        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    
    % ビデオの保存
    close(v);
    
end


function B = reorderMatrix2(A)
    idx = find(mod(1:length(A), 6) == 1 | mod(1:length(A), 6) == 2 | mod(1:length(A), 6) == 3);
    A = flip(A(idx));
    B = [];
    %A = 1:100;
    n = 3;
    for i = 1:n:length(A)
        sub_vector = A(i:min(i+n-1, length(A)));
        B = [B; flip(sub_vector)];
    end
end