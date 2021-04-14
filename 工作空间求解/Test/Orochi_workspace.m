%%%%%%%%%%%%%%%%%% 7DOF协作机器人连杆模型构建与工作空间求解 %%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/10/9
% DH: MDH or SDH
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 机器人连杆参数
tic
clear all
close all
clc

global md i
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
i = 50000;
% MDH = 1; SDH = 0;
md = 1;
if md == 1
    % 修正DH参数
    %           theta  d     a     alpha  offset
    L(1) = Link([0     0     0     0      0],'modified');
    L(2) = Link([0     0     0     -pi/2  0],'modified');
    L(3) = Link([0     0.5   0     pi/2   0],'modified');
    L(4) = Link([0     0     0     -pi/2  0],'modified');
    L(5) = Link([0     0.4   0     pi/2   0],'modified');
    L(6) = Link([0     0     0     -pi/2  0],'modified');
    L(7) = Link([0     0     0     pi/2   0],'modified');
    nym_Link = SerialLink(L,'name','nymrobot');
    % nym_Link.plot([0 -pi/2 pi 0 0 0 0])
    % h = nymfkine(0,0,0,0,0,0,0);            % 自写正解函数求解角度
    % h1 = nym_Link.fkine([0 0 0 0 0 0 0]);   % Robotics toolbox正解函数角度
    % teach(nym_Link)
elseif md == 0
    % 标准DH参数
    %           theta  d     a     alpha  offset
    L(1) = Link([0     0     0     -pi/2  0]);
    L(2) = Link([0     0     0     pi/2   0]);
    L(3) = Link([0     0.5   0     -pi/2  0]);
    L(4) = Link([0     0     0     pi/2   0]);
    L(5) = Link([0     0.4   0     -pi/2  0]);
    L(6) = Link([0     0     0     pi/2   0]);
    L(7) = Link([0     0.1   0     0      0]);
    nym_Link = SerialLink(L,'name','nymrobot');
    nym_Link.plot([0 -pi/2 pi 0 0 0 0])
    % h = nymfkine(0,-pi/2,pi,0,0,0,0);            % 自写正解函数求解角度
    % h1 = nym_Link.fkine([0 -pi/2 pi 0 0 0 0]);   % Robotics toolbox正解函数角度
    % q2=nym_Link.ikine(h1)
    % teach(nym_Link)
end

%% 角度参数
theta1 = 0; 
theta2 = pi/4; 
theta3 = pi; 
theta4 = pi/4; 
theta5 = 0; 
theta6 = 0; 
theta7 = 0;
t = Orochi_fkine(theta1,theta2,theta3,theta4,theta5,theta6,theta7);           % 自写正解函数求解角度
t1 = nym_Link.fkine([theta1 theta2 theta3 theta4 theta5 theta6 theta7]);  % Robotics toolbox正解函数角度
% nym_Link.plot([0 pi/4 pi pi/2 0 0 0])
% teach(nym_Link) % 示教界面

%% 蒙特卡洛方法求解工作空间
A = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第一关节变量限位
B = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第二关节变量限位
C = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第三关节变量限位
D = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第四关节变量限位
E = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % 第五关节变量限位
F = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % 第六关节变量限位
G = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % 第七关节变量限位

% 总工作空间
Q1 = cell(i,7);                                       % 建立元胞数组
for n = 1:i
    Q1{n} = [A(n) B(n) C(n) D(n) E(n) F(n) G(n)];     % 完整工作空间
%     Q1{n} = [A(n) B(n) 0 0 0 0 0];                  % 边界空间，运动时应规避
end                                                   % 产生30000组随机点
H1 = cell2mat(Q1);                                    % 将元胞数组转化为矩阵
T1 = double(nym_Link.fkine(H1));                      % 机械臂正解
a1 = zeros(1,i);

% 可操作工作空间X-Z截面
Q2 = cell(i,7);                                       % 建立元胞数组
for n = 1:i
    Q2{n} = [0  B(n) 0  D(n) 0  F(n) 0];              % 可操作工作空间X-Z截面
end                                                   % 产生30000组随机点
H2 = cell2mat(Q2);                                    % 将元胞数组转化为矩阵
T2 = double(nym_Link.fkine(H2));                      % 机械臂正解
a2 = zeros(1,i); 

% 可操作工作空间X-Y截面
Q3 = cell(i,7);                                       % 建立元胞数组
for n = 1:i
    Q3{n} = [A(n) pi/2 pi/2 D(n) E(n) F(n) G(n)];     % 可操作工作空间X-Y截面
end                                                   % 产生30000组随机点
H3 = cell2mat(Q3);                                    % 将元胞数组转化为矩阵
T3 = double(nym_Link.fkine(H3));                      % 机械臂正解
a3 = zeros(1,i);

%% 画图
figure(1)
scatter3(squeeze(T1(1,4,:)),squeeze(T1(2,4,:)),squeeze(T1(3,4,:)))                         % 随机点图
% teach(nym_Link) % 示教界面
figure(2)
scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)))                         % 随机点图
nym_Link.plot([0 0 0 0 0 0 0])
figure(3)
scatter3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)))                         % 随机点图
nym_Link.plot([0 0 0 0 0 0 0])

%%%  离散化工作空间  %%%
% nym_Link.plot([-pi pi 0 -pi pi/2 pi -pi/2],'workspace',[-1.5 1.5 -1.5 1.5 -1.5 1.5])     % 机械臂图
% figure(4)
% for k=1:i
%     Orochi_plotcube([0.1 0.1 0.1],[squeeze(T1(1,4,k)) squeeze(T1(2,4,k)) squeeze(T1(3,4,k))],.8,[0 0.7 1])
% end
% grid on
toc
