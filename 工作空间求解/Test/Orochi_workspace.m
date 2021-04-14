%%%%%%%%%%%%%%%%%% 7DOFЭ������������ģ�͹����빤���ռ���� %%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/10/9
% DH: MDH or SDH
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ���������˲���
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
    % ����DH����
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
    % h = nymfkine(0,0,0,0,0,0,0);            % ��д���⺯�����Ƕ�
    % h1 = nym_Link.fkine([0 0 0 0 0 0 0]);   % Robotics toolbox���⺯���Ƕ�
    % teach(nym_Link)
elseif md == 0
    % ��׼DH����
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
    % h = nymfkine(0,-pi/2,pi,0,0,0,0);            % ��д���⺯�����Ƕ�
    % h1 = nym_Link.fkine([0 -pi/2 pi 0 0 0 0]);   % Robotics toolbox���⺯���Ƕ�
    % q2=nym_Link.ikine(h1)
    % teach(nym_Link)
end

%% �ǶȲ���
theta1 = 0; 
theta2 = pi/4; 
theta3 = pi; 
theta4 = pi/4; 
theta5 = 0; 
theta6 = 0; 
theta7 = 0;
t = Orochi_fkine(theta1,theta2,theta3,theta4,theta5,theta6,theta7);           % ��д���⺯�����Ƕ�
t1 = nym_Link.fkine([theta1 theta2 theta3 theta4 theta5 theta6 theta7]);  % Robotics toolbox���⺯���Ƕ�
% nym_Link.plot([0 pi/4 pi pi/2 0 0 0])
% teach(nym_Link) % ʾ�̽���

%% ���ؿ��巽����⹤���ռ�
A = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ��һ�ؽڱ�����λ
B = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �ڶ��ؽڱ�����λ
C = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % �����ؽڱ�����λ
D = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % ���Ĺؽڱ�����λ
E = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ����ؽڱ�����λ
F = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �����ؽڱ�����λ
G = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % ���߹ؽڱ�����λ

% �ܹ����ռ�
Q1 = cell(i,7);                                       % ����Ԫ������
for n = 1:i
    Q1{n} = [A(n) B(n) C(n) D(n) E(n) F(n) G(n)];     % ���������ռ�
%     Q1{n} = [A(n) B(n) 0 0 0 0 0];                  % �߽�ռ䣬�˶�ʱӦ���
end                                                   % ����30000�������
H1 = cell2mat(Q1);                                    % ��Ԫ������ת��Ϊ����
T1 = double(nym_Link.fkine(H1));                      % ��е������
a1 = zeros(1,i);

% �ɲ��������ռ�X-Z����
Q2 = cell(i,7);                                       % ����Ԫ������
for n = 1:i
    Q2{n} = [0  B(n) 0  D(n) 0  F(n) 0];              % �ɲ��������ռ�X-Z����
end                                                   % ����30000�������
H2 = cell2mat(Q2);                                    % ��Ԫ������ת��Ϊ����
T2 = double(nym_Link.fkine(H2));                      % ��е������
a2 = zeros(1,i); 

% �ɲ��������ռ�X-Y����
Q3 = cell(i,7);                                       % ����Ԫ������
for n = 1:i
    Q3{n} = [A(n) pi/2 pi/2 D(n) E(n) F(n) G(n)];     % �ɲ��������ռ�X-Y����
end                                                   % ����30000�������
H3 = cell2mat(Q3);                                    % ��Ԫ������ת��Ϊ����
T3 = double(nym_Link.fkine(H3));                      % ��е������
a3 = zeros(1,i);

%% ��ͼ
figure(1)
scatter3(squeeze(T1(1,4,:)),squeeze(T1(2,4,:)),squeeze(T1(3,4,:)))                         % �����ͼ
% teach(nym_Link) % ʾ�̽���
figure(2)
scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)))                         % �����ͼ
nym_Link.plot([0 0 0 0 0 0 0])
figure(3)
scatter3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)))                         % �����ͼ
nym_Link.plot([0 0 0 0 0 0 0])

%%%  ��ɢ�������ռ�  %%%
% nym_Link.plot([-pi pi 0 -pi pi/2 pi -pi/2],'workspace',[-1.5 1.5 -1.5 1.5 -1.5 1.5])     % ��е��ͼ
% figure(4)
% for k=1:i
%     Orochi_plotcube([0.1 0.1 0.1],[squeeze(T1(1,4,k)) squeeze(T1(2,4,k)) squeeze(T1(3,4,k))],.8,[0 0.7 1])
% end
% grid on
toc
