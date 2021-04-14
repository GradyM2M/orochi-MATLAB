%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�������˹ؽڿռ�켣�滮 %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/01/12
% DH: SDH
% ������ʼ�ؽڽǶȺ���ֹ�ؽڽǶ�
% [q,qd,qdd] = jtraj(q0,qf,m);       %������ζ���ʽ�滮�켣
% tc = ctraj(T0,T1,n);               %�����ȼ����ȼ��ٹ滮�켣
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

%�����˽�ģ
L1 = Link([0     0     0     -pi/2  0]);
L2 = Link([0     0     0     pi/2   0]);
L3 = Link([0     0.5   0     -pi/2  0]);
L4 = Link([0     0     0     pi/2   0]);
L5 = Link([0     0.4   0     -pi/2  0]);
L6 = Link([0     0     0     pi/2   0]);
L7 = Link([0     0.1   0     0      0]);
%�����˴������
robot = SerialLink([L1,L2,L3,L4,L5,L6,L7],'name','Orochi');
% robot.name = '7���ɶ�Э��������';
% robot.manuf = 'Orochi';


%******************%
%****** jtraj ******%
%******************%
% �켣�滮��������
init_ang = [0 0 0 0 0 0 0];
targ_ang = [pi/4, pi/2, pi/4, pi/4, 0, 0, 0];
step = 50;

%�켣�滮����
[q,qd,qdd] = jtraj(init_ang,targ_ang,step);
%������ʾ
figure(1)
subplot(3,2,[1,3]); 
T = double(robot.fkine(q));%�˶�ѧ����
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%���ĩ�˹켣
grid on
robot.plot(q);
%��ʾλ�á��ٶȡ����ٶȱ仯����
subplot(3, 2, 2);
i = 1:6;
plot(q(:,1));
title('λ��');
grid on;

subplot(3, 2, 4);
i = 1:6;
plot(qd(:,1));
title('�ٶ�');
grid on;

subplot(3, 2, 6);
i = 1:6;
plot(qdd(:,1));
title('���ٶ�');
grid on;

T = double(robot.fkine(q));%�˶�ѧ����
subplot(3,2,5);
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%���ĩ�˹켣
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
% T = robot.fkine(q);%�˶�ѧ����
% plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)));%���ĩ�˹켣
% grid on
% robot.plot(q);