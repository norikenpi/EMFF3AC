clc; close all; clear all
%% NN model constants
load("NN_optimal_power_1212")
Weight_nor_label = Weight_nor_label_power;
Bias_nor_label = Bias_nor_label_power;
constants = cell(7,1);
constants{1}=Bias_nor_input;
constants{2}=Weight_nor_input;
constants{3}=weight1;
constants{4}=offset1;
constants{5}=weight2;
constants{6}=Weight_nor_label;
constants{7}=Bias_nor_label;
%% random input
mu_0=4*pi*10^(-7);
d_max = 1; d_min = 0.1;
f_max = 1e-7; f_min = 1e-20; % for norm(f)~=0
tau_max = 3.5e-7; tau_min = 1e-20;% for norm(tau)~=0
[input,Control_LOS,d] = states_gene(d_max,d_min,f_max,f_min,tau_max,tau_min);


% convex optimization
EM_con = (mu_0)/(8*pi*d^4);
[power_per_EM_con,l_cvx] = cvx_optimization(d,Control_LOS,EM_con);
power_per_EM_con
% prediction
predicted_optimal_power = prediction(input,constants)


function predicted_optimal_power = prediction(input,constants)
    Bias_nor_input=constants{1};
    Weight_nor_input=constants{2};
    weight1=constants{3};
    offset1=constants{4};
    weight2=constants{5};
    Weight_nor_label=constants{6};
    Bias_nor_label=constants{7};
    nor_input = (input- Bias_nor_input)*inv(Weight_nor_input);
    nor_label = double(weight2*max(weight1*nor_input.'+offset1, 0));
    predicted_optimal_power = Weight_nor_label*nor_label + Bias_nor_label;
end

function [power_per_EM_con,l] = cvx_optimization(d,Control_LOS,EM_con)
    cvx_begin sdp quiet
    variable l1 %nonnegative
    variable l2 %nonnegative
    variable l3 %nonnegative
    variable l4 %nonnegative
    variable l5 %nonnegative
    variable l6 %nonnegative
    power_per_EM_con = -[l1,l2,l3,l4,l5,l6]*Control_LOS/EM_con;
    minimize -power_per_EM_con
    subject to
    Rl = [-6*l1, 3*l2-d*l6,3*l3+d*l5;
        3*l2-2*d*l6,3*l1,-d*l4;
        3*l3+2*d*l5,d*l4,3*l1];
    [eye(3),Rl;Rl.',eye(3)]>=0;
    %-[l1,l2,l3,l4,l5,l6]*Control_LOS >=0
    cvx_end
    l = 1/EM_con*[l1;l2;l3;l4;l5;l6];
end

function [input,Control_LOS,d] = states_gene(d_max,d_min,f_max,f_min,tau_max,tau_min)
    if 1
        %
        d_sub = 1/sqrt(3)*(d_max-d_min)*(2*rand(3,1)-1);
        r10 = d_sub + d_min*d_sub/norm(d_sub);
        d = norm(r10);
        f_sub = 1/sqrt(3)*(f_max-f_min)*(2*rand(3,1)-1);
        force = f_sub + f_min*f_sub/norm(f_sub);
        if 1
            tau_sub = 1/sqrt(3)*(tau_max-tau_min)*(2*rand(3,1)-1);
            torque = tau_sub + tau_min*tau_sub/norm(tau_sub);
            Control = [force;torque];
            input = [r10;Control].';
            % new C_LOS
            C_los2o = C_LOS(r10,Control);
            r_los = C_los2o.'*r10;
            Control_LOS = [C_los2o.'*Control(1:3,1);C_los2o.'*Control(4:6,1)];
        else
            torque = (-1/2)*cross(r10,force);
            Control = [force;torque];
            % new C_LOS
            C_los2o = C_LOS(r10,Control);
            r_los = C_los2o.'*r10;
            Control = [force;torque];
            Control_LOS = [C_los2o.'*Control(1:3,1);C_los2o.'*Control(4:6,1)];
            Control_LOS(3)=0;Control_LOS(4)=0;Control_LOS(5)=0;
            input = [d,Control_LOS(1:2,:).',Control_LOS(4:6,:).'];
            input = input(1:3);
            %Inputs = [Inputs; [d,Control_LOS(1:2,:).',Control_LOS(4:6,:).']];
        end
        % elseif 1
        %     r10 = Position(num,:).';
        %     torque = (-1/2)*cross(r10,Force(num,:).');
        %     Control = [Force(num,:).';torque];
    end
end
function out = fun_Control_LOS(u,EM_con,Q)
    out=EM_con*Q*(kron([u(1);u(2);u(3)],[u(7);u(8);u(9)])+...
        kron([u(4);u(5);u(6)],[u(10);u(11);u(12)]));
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