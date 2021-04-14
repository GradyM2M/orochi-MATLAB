%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人工作空间臂长优化 %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 机器人连杆参数
clear 
close all
clc
global i 
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
i = 100000;
% j = 50;

% %% 角度参数
% theta1 = 0; 
% theta2 = pi/4; 
% theta3 = pi; 
% theta4 = pi/4; 
% theta5 = 0; 
% theta6 = 0; 
% theta7 = 0;
% t1 = nym_Link.fkine([theta1 theta2 theta3 theta4 theta5 theta6 theta7]);  % Robotics toolbox正解函数角度

%% 连杆限位
d1 = 0.1:0.05:0.8;    % 连杆1限位
d2 = 0.9-d1;          % 连杆2限位
d3 = 0.1;

%% 关节角度限位
A = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第一关节变量限位
B = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第二关节变量限位
C = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第三关节变量限位
D = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第四关节变量限位
E = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第五关节变量限位
F = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第六关节变量限位
G = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % 第七关节变量限位

%% 蒙特卡洛方法求解工作空间

for n = 1:length(d1)
    % 标准DH参数
    %           theta  d     a     alpha  offset
    L(1) = Link([0     0     0     -pi/2  0]);
    L(2) = Link([0     0     0     pi/2   0]);
    L(3) = Link([0     d1(n) 0     -pi/2  0]);
    L(4) = Link([0     0     0     pi/2   0]);
    L(5) = Link([0     d2(n) 0     -pi/2  0]);
    L(6) = Link([0     0     0     pi/2   0]);
    L(7) = Link([0     d3    0     0      0]);
    nym_Link = SerialLink(L,'name','nymrobot');
    % nym_Link.plot([0 -pi/2 pi 0 0 0 0])

    %% 可操作工作空间X-Z截面
    Q = cell(i,7);                                   % 建立元胞数组
    Q1 = zeros(i,7);
    for k = 1:i
        Q{k} = [0  B(k) 0  D(k) 0  F(k) 0];          % 可操作工作空间X-Z截面
    end                                              % 产生30000组随机点
    H2 = cell2mat(Q);                                % 将元胞数组转化为矩阵
    T2 = double(nym_Link.fkine(H2));                 % 机械臂正解
    T3= double(nym_Link.fkine(Q1));                  % 机械臂正解
    
    %% 画图
    figure(n)
    scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T3(3,4,:)))  % 随机点图
    grid off
    axis off
end
