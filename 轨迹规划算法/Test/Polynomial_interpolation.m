%%%%%%%%%%%%%%%%%%%%%%%%%% ��ζ���ʽ��ֵ���߲��� %%%%%%%%%%%%%%%%%%%%%%%%%%%
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

figure('Name','��ζ���ʽ��ֵ����')
subplot(3,1,1);
plot(t,s)
grid on
xlabel('ʱ��(s)');ylabel('theta7λ��(rad)');
xlim([0,ts]);
subplot(3,1,2);
plot(t,sd);
grid on
xlabel('ʱ��(s)');ylabel('theta7�ٶ�(rad/s)');
xlim([0,ts]);
subplot(3,1,3);
plot(t,sdd)
grid on
xlabel('ʱ��(s)');ylabel('theta7���ٶ�(rad/s^2)');
xlim([0,ts]);

