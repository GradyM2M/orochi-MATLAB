%%%%%%%%%%%%%%%%%%%%%%%%%% 五次多项式插值曲线测试 %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
s0 = 0;
sf = pi;
ts = 5;
t = 0:0.01:ts;
[s,sd,sdd] = tpoly(s0,sf,t);

figure('Name','五次多项式插值曲线')
subplot(3,1,1);
plot(t,s)
grid on
xlabel('时间(s)');ylabel('theta7位移(rad)');
xlim([0,ts]);
subplot(3,1,2);
plot(t,sd);
grid on
xlabel('时间(s)');ylabel('theta7速度(rad/s)');
xlim([0,ts]);
subplot(3,1,3);
plot(t,sdd)
grid on
xlabel('时间(s)');ylabel('theta7加速度(rad/s^2)');
xlim([0,ts]);

