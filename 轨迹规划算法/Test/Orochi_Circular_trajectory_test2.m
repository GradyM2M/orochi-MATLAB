%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人关节空间轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/01/12
% DH: SDH
% 给定起始位置坐标和终止位置坐标
% [q,qd,qdd] = jtraj(q0,qf,m);       %利用五次多项式规划轨迹
% tc = ctraj(T0,T1,n);               %利用匀加速匀减速规划轨迹
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc;
%建立机器人模型
%          theta d     a     alpha  offset
L1 = Link([0     0     0     -pi/2  0]);
L2 = Link([0     0     0     pi/2   0]);
L3 = Link([0     0.5   0     -pi/2  0]);
L4 = Link([0     0     0     pi/2   0]);
L5 = Link([0     0.4   0     -pi/2  0]);
L6 = Link([0     0     0     pi/2   0]);
L7 = Link([0     0.1   0     0      0]);
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','Orochi'); %连接连杆

T1=transl(0,0,1);%根据给定起始点，得到起始点位姿
T2=transl(0.5,0.5,0.2);%根据给定终止点，得到终止点位姿


q1=robot.ikine(T1);%根据起始点位姿，得到起始点关节角
q2=robot.ikine(T2);%根据终止点位姿，得到终止点关节角
[q,qd,qdd]=jtraj(q1,q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
grid on
T=double(robot.fkine(q));%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'b');%输出末端轨迹
hold on
robot.plot(q);%动画演示
hold on

T1=transl(-0.6,0.1,-0.4);%根据给定起始点，得到起始点位姿
T2=transl(0.5,0.5,0.2);%根据给定终止点，得到终止点位姿
Q1=robot.ikine(T2);%根据起始点位姿，得到起始点关节角
Q2=robot.ikine(T1);%根据终止点位姿，得到终止点关节角
[q,qd,qdd]=jtraj(Q1,Q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
T=double(robot.fkine(q));%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'r');%输出末端轨迹
robot.plot(q);%动画演示
hold on 

T1=transl(-0.6,0.1,-0.4);%根据给定起始点，得到起始点位姿
T2=transl(0.6,0.1,0.4);%根据给定终止点，得到终止点位姿
q1=robot.ikine(T1);%根据起始点位姿，得到起始点关节角
q2=robot.ikine(T2);%根据终止点位姿，得到终止点关节角
[q,qd,qdd]=jtraj(q1,q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
T=double(robot.fkine(q));%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'g');%输出末端轨迹
hold on 
robot.plot(q);%动画演示

T1=transl(0,0,1);%根据给定起始点，得到起始点位姿
T2=transl(0.6,0.1,0.4);%根据给定终止点，得到终止点位姿
Q1=robot.ikine(T2);%根据起始点位姿，得到起始点关节角
Q2=robot.ikine(T1);%根据终止点位姿，得到终止点关节角
[q,qd,qdd]=jtraj(Q1,Q2,50); %五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
T=double(robot.fkine(q));%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'k');%输出末端轨迹
robot.plot(q);%动画演示
