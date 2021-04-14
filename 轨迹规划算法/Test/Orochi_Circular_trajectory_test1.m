%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人关节空间轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/01/12
% DH: SDH
% 给定初始关节角度和终止关节角度
% [q,qd,qdd] = jtraj(q0,qf,m);       %利用五次多项式规划轨迹
% tc = ctraj(T0,T1,n);               %利用匀加速匀减速规划轨迹
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

%机器人建模
L1 = Link([0     0     0     -pi/2  0]);
L2 = Link([0     0     0     pi/2   0]);
L3 = Link([0     0.5   0     -pi/2  0]);
L4 = Link([0     0     0     pi/2   0]);
L5 = Link([0     0.4   0     -pi/2  0]);
L6 = Link([0     0     0     pi/2   0]);
L7 = Link([0     0.1   0     0      0]);
%机器人搭建与命名
robot = SerialLink([L1,L2,L3,L4,L5,L6,L7],'name','Orochi');
% robot.name = '7自由度协作机器人';
% robot.manuf = 'Orochi';


%******************%
%****** jtraj ******%
%******************%
% 轨迹规划参数设置
init_ang = [0 0 0 0 0 0 0];
targ_ang = [pi/4, pi/2, pi/4, pi/4, 0, 0, 0];
step = 50;

%轨迹规划方法
[q,qd,qdd] = jtraj(init_ang,targ_ang,step);
%动画显示
figure(1)
subplot(3,2,[1,3]); 
T = double(robot.fkine(q));%运动学正解
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%输出末端轨迹
grid on
robot.plot(q);
%显示位置、速度、加速度变化曲线
subplot(3, 2, 2);
i = 1:6;
plot(q(:,1));
title('位置');
grid on;

subplot(3, 2, 4);
i = 1:6;
plot(qd(:,1));
title('速度');
grid on;

subplot(3, 2, 6);
i = 1:6;
plot(qdd(:,1));
title('加速度');
grid on;

T = double(robot.fkine(q));%运动学正解
subplot(3,2,5);
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%输出末端轨迹
grid on

figure(2)
subplot(3,1,1)
plot(squeeze(T(1,4,:)));
grid on
title('Px');
subplot(3,1,2)
plot(squeeze(T(2,4,:)));
grid on
title('Py');
subplot(3,1,3)
plot(squeeze(T(3,4,:)));
grid on
title('Pz');

% figure(3)
% T = robot.fkine(q);%运动学正解
% plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%输出末端轨迹
% grid on
% robot.plot(q);