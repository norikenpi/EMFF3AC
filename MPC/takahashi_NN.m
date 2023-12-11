clc; close all; clear all
load("NN_optimal_power") % ニューラルネットワークのパラメータを読み込む

% 相対距離と相対力の上限下限を定義
d_max = 1; d_min = 0.2;
f_max = 1e-7; f_min = 1e-10;

% ランダムな相対距離を生成
d_sub = 1/sqrt(3)*(d_max-d_min)*(2*rand(3,1)-1); % -1から1の範囲のランダムな3次元ベクトルを生成
r10 = d_sub + d_min*d_sub/norm(d_sub); % 最小距離を保証しつつ、ランダムな距離ベクトルを調整
d = norm(r10); % ベクトルr10の長さ（ノルム）を計算

% ランダムな相対力を生成
f_sub = 1/sqrt(3)*(f_max-f_min)*(2*rand(3,1)-1); % -1から1の範囲のランダムな3次元ベクトルを生成
force = f_sub + f_min*f_sub/norm(f_sub); % 最小力を保証しつつ、ランダムな力ベクトルを調整

% 制御情報を計算
Control_LOS = control_into_LOS(r10,force);

% ニューラルネットワークの入力を準備
input = [d,Control_LOS(1:2,:).',Control_LOS(4:6,:).'];
input = input(1:3);

% 入力データの正規化
nor_input = (input - Bias_nor_input) * inv(Weight_nor_input);

% ニューラルネットワークを使って予測を行う
% 重みとバイアスを用いて、非線形の関係をモデル化
nor_label = weight2 * tanh(weight1 * nor_input.' + offset1) + offset2;

% 最適な力を予測 nor_labelは正規化された出力に対して適用される係数。
predicted_optimal_power = double(Weight_nor_label * nor_label + Bias_nor_label)

% 以下のコードは最適化問題を解くためのもの

mu_0=4*pi*10^(-7);
cvx_begin sdp quiet
variable l1 nonnegative
variable l2 nonnegative
variable l3 nonnegative
variable l4 nonnegative
variable l5 nonnegative
variable l6 nonnegative
power = -1/mu_0*[l1,l2,l3,l4,l5,l6]*Control_LOS;
minimize -power
subject to
    Rl = [-6*l1, 3*l2-d*l6,3*l3+d*l5;
        3*l2-2*d*l6,3*l1,-d*l4;
        3*l3+2*d*l5,d*l4,3*l1];
    [eye(3),Rl;Rl.',eye(3)]>=0;
cvx_end

caluculated_optimal_power = power

function Control_LOS = control_into_LOS(r10,force)
    % 座標変換してそう
    torque = (-1/2)*cross(r10,force);
    Control = [force;torque];
    
    d = norm(r10);
    C_los2o = C_LOS(r10,Control);
    r_los = C_los2o.'*r10;
    Control_LOS = [C_los2o.'*Control(1:3,1);C_los2o.'*Control(4:6,1)];
    %
    Control_LOS(3)=0;Control_LOS(4)=0;Control_LOS(5)=0;
end
function C_los2o = C_LOS(r10,Control)
    force = Control(1:3,1);
    e_x = r10/norm(r10);
    s = force-dot(e_x,force)*e_x;
    e_y = s/norm(s);
    e_z = cross(e_x,e_y);
    C_LOS_2_A = [e_x,e_y,e_z];
    C_los2o = C_LOS_2_A;
end