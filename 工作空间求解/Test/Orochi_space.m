%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�������˹����ռ�۳��Ż� %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ���������˲���
clear 
close all
clc
global i 
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
i = 100000;
% j = 50;

% %% �ǶȲ���
% theta1 = 0; 
% theta2 = pi/4; 
% theta3 = pi; 
% theta4 = pi/4; 
% theta5 = 0; 
% theta6 = 0; 
% theta7 = 0;
% t1 = nym_Link.fkine([theta1 theta2 theta3 theta4 theta5 theta6 theta7]);  % Robotics toolbox���⺯���Ƕ�

%% ������λ
d1 = 0.1:0.05:0.8;    % ����1��λ
d2 = 0.9-d1;          % ����2��λ
d3 = 0.1;

%% �ؽڽǶ���λ
A = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ��һ�ؽڱ�����λ
B = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �ڶ��ؽڱ�����λ
C = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % �����ؽڱ�����λ
D = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % ���Ĺؽڱ�����λ
E = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ����ؽڱ�����λ
F = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �����ؽڱ�����λ
G = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % ���߹ؽڱ�����λ

%% ���ؿ��巽����⹤���ռ�

for n = 1:length(d1)
    % ��׼DH����
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

    %% �ɲ��������ռ�X-Z����
    Q = cell(i,7);                                   % ����Ԫ������
    Q1 = zeros(i,7);
    for k = 1:i
        Q{k} = [0  B(k) 0  D(k) 0  F(k) 0];          % �ɲ��������ռ�X-Z����
    end                                              % ����30000�������
    H2 = cell2mat(Q);                                % ��Ԫ������ת��Ϊ����
    T2 = double(nym_Link.fkine(H2));                 % ��е������
    T3= double(nym_Link.fkine(Q1));                  % ��е������
    
    %% ��ͼ
    figure(n)
    scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T3(3,4,:)))  % �����ͼ
    grid off
    axis off
end
