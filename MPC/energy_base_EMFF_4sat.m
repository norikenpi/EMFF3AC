
clear
%% パラメータ設定
% 最終衛星間距離
d_target = 0.925;

% 衛星数　2基or5基or9基
num = 4;

% 衛星質量
m = 1; % 1
%m = 0.38; % 1
 
% タイムステップ(s)
dt = 10;

% 時間 シミュレーション時間はN×dt秒250
N = 10;
N = 1;
%u_max = 1e-9;
coilN = 140;
radius = 0.05;
P_max = 10; % W
rho = 1.68e-7; % Ω/m
wire_length = 140*0.05*2*pi;
wire_S = (0.2e-3)^2*pi;
R_rho = rho * wire_length/wire_S; 
I_max = sqrt(P_max/R_rho);
myu_max = I_max * coilN * radius^2 * pi;
disp("最大電力設定")
disp(P_max)
disp("最大電流設定")
disp(I_max)
disp("最大磁気モーメント設定")
disp(myu_max)

d_avoid = radius*6;
% 初期衛星間距離
d_initial = d_avoid/2;

rr1 = d_target/(2*sqrt(3));
rr2 = sqrt(2)*d_target/2;
rr = [rr1,rr2];
rr1 = rr(1);

satellites = cell(1, num);
s01 = [-d_initial; -d_initial; 0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s02 = [d_initial; d_initial; -0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s03= [-d_initial; d_initial; 0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s04 = [d_initial; -d_initial; -0.00005; (2*rand-1)*1e-5; (2*rand-1)*1e-5; (2*rand-1)*1e-5];
s0 = adjust_cog([s01, s02, s03, s04], num); % 6num×1

%各衛星初期状態決定
s01 = s0(1:6);
s02 = s0(7:12);
s03 = s0(13:18);
s04 = s0(19:24);

E_border = 0.1;
E_all = 100;

state1 = zeros(N,6);
state2 = zeros(N,6);
state3 = zeros(N,6);
state4 = zeros(N,6);

state1(1,:) = s01.';
state2(1,:) = s02.';
state3(1,:) = s03.';
state4(1,:) = s04.';

%% シミュレーション
%エネルギーの総和がE_border以下だったら問題ない。
while E_all > E_border
    % ペア組み

    % 最大電力割り当て
    % ペアごとに割り振る感じがいいのかな。

    % 各ペア制御入力計算（for ペアlist　座標平行移動）
    % ペアでforを回して、一発で現在の状態と最大磁気モーメントから制御入力とペア間に働く力が出る関数がほしい。
    % 最適化ベースになるとここが変わるだけ。

    % 推力計算
    % ペアでforを回して、各衛星に働く力を計算。

    % 状態量更新
    % 現在の状態量と推力を入れたら次の時刻の状態量が出てくる関数
    satellites{1} = state1(:,1:3);
    satellites{2} = state2(:,1:3);
    satellites{3} = state3(:,1:3);
    satellites{4} = state4(:,1:3);

    %総エネルギー計算
    %次の時刻の状態から総エネルギーを計算
    E_all = 0.1;
end
disp("シミュレーション終了")

%% 図示

plot_s(satellites, num, N, rr, d_target)

%% 関数リスト
function s = adjust_cog(s_mat, num)
    
    cog = sum(s_mat, 2)/num;
    s = zeros(6*num, 1);
    for i = 1:num
        s(6*(i-1)+1: 6*i) = s_mat(:,i) - cog;
    end
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
    for i = 1:1:N
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