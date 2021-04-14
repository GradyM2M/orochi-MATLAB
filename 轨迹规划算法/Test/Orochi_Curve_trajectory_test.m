clear all
close all
clc;
%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�������˹ؽڿռ�켣�滮 %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/01/12
% DH: SDH
% ������ʼ�ؽڽǶȺ���ֹ�ؽڽǶ�
% [q,qd,qdd] = jtraj(q0,qf,m);       %������ζ���ʽ�滮�켣
% tc = ctraj(T0,T1,n);               %�����ȼ����ȼ��ٹ滮�켣
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%����������ģ��
%          theta d     a     alpha  offset
L1 = Link([0     0     0     -pi/2  0]);
L2 = Link([0     0     0     pi/2   0]);
L3 = Link([0     0.5   0     -pi/2  0]);
L4 = Link([0     0     0     pi/2   0]);
L5 = Link([0     0.4   0     -pi/2  0]);
L6 = Link([0     0     0     pi/2   0]);
L7 = Link([0     0.1   0     0      0]);
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','Orochi'); %��������

T1=transl(0,0,1);%���ݸ�����ʼ�㣬�õ���ʼ��λ��
T2=transl(0.5,0.1,0.5);%���ݸ�����ֹ�㣬�õ���ֹ��λ��


q1=robot.ikine(T1);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q2=robot.ikine(T2);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q1,qd1,qdd1]=jtraj(q1,q2,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
grid on
T=double(robot.fkine(q1));%���ݲ�ֵ���õ�ĩ��ִ����λ��
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'b');%���ĩ�˹켣
hold on
robot.plot(q1);%������ʾ
hold on 
Q1=robot.ikine(T2);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
Q2=robot.ikine(T1);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q2,qd2,qdd2]=jtraj(Q1,Q2,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
robot.plot(q2);%������ʾ
hold on 
q1=robot.ikine(T1);%������ʼ��λ�ˣ��õ���ʼ��ؽڽ�
q2=robot.ikine(T2);%������ֹ��λ�ˣ��õ���ֹ��ؽڽ�
[q3,qd3,qdd3]=jtraj(q1,q2,50); %��ζ���ʽ�켣���õ��ؽڽǶȣ����ٶȣ��Ǽ��ٶȣ�50Ϊ���������
T=double(robot.fkine(q3));%���ݲ�ֵ���õ�ĩ��ִ����λ��
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'r');%���ĩ�˹켣
hold on 
robot.plot(q3);%������ʾ