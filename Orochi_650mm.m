clear
clc
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% D-H����
L(1) = Link([0     0     0     -pi/2  0]);
L(2) = Link([0     0     0     pi/2   0]);
L(3) = Link([0     0.35  0     -pi/2  0]);
L(4) = Link([0     0     0     pi/2   0]);
L(5) = Link([0     0.3   0     -pi/2  0]);
L(6) = Link([0     0     0     pi/2   0]);
L(7) = Link([0     0.1   0     0      0]);
Orochi = SerialLink(L,'name','650mm��Ч�۳�');
Orochi.plot([0 0 0 0 0 0 0])

i = 10000;

%% ���ؿ��巽����⹤���ռ�
A = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ��һ�ؽڱ�����λ
B = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �ڶ��ؽڱ�����λ
C = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % �����ؽڱ�����λ
D = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % ���Ĺؽڱ�����λ
E = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ����ؽڱ�����λ
F = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �����ؽڱ�����λ
G = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % ���߹ؽڱ�����λ

%% �ܹ����ռ�
Q1 = cell(i,7);                                   % ����Ԫ������
for n = 1:i
    Q1{n} = [A(n) B(n) C(n) D(n) E(n) F(n) G(n)]; % ���������ռ�
end                                               % ����30000�������
H1 = cell2mat(Q1);                                % ��Ԫ������ת��Ϊ����
T1 = double(Orochi.fkine(H1));                    % ��е������

Q2 = cell(i,7);                                   % ����Ԫ������
for n = 1:i
    Q2{n} = [A(n) B(n) 0 0 0 0 0];                % �߽�ռ䣬�˶�ʱӦ���
end                                               % ����30000�������
H2 = cell2mat(Q2);                                % ��Ԫ������ת��Ϊ����
T2 = double(Orochi.fkine(H2));                    % ��е������

% �ɲ��������ռ�X-Z����
Q3 = cell(i,7);                                   % ����Ԫ������
for n = 1:i
    Q3{n} = [0  B(n) 0  D(n) 0  F(n) 0];          % �ɲ��������ռ�X-Z����
end                                               % ����30000�������
H3 = cell2mat(Q3);                                % ��Ԫ������ת��Ϊ����
T3 = double(Orochi.fkine(H3));                    % ��е������

% �ɲ��������ռ�X-Y����
Q4 = cell(i,7);                                   % ����Ԫ������
for n = 1:i
    Q4{n} = [A(n) pi/2 pi/2 D(n) E(n) F(n) G(n)]; % �ɲ��������ռ�X-Y����
end                                               % ����30000�������
H4 = cell2mat(Q4);                                % ��Ԫ������ת��Ϊ����
T4 = double(Orochi.fkine(H4));                    % ��е������

%% ��ͼ
figure(1)
scatter3(squeeze(T1(1,4,:)),squeeze(T1(2,4,:)),squeeze(T1(3,4,:)))% �����ͼ
Orochi.plot([0 0 0 0 0 0 0])
axis([-1 1 -1 1 -0.6 1])
pause(1)
figure(2)
scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)))% �����ͼ
Orochi.plot([0 0 0 0 0 0 0])
axis([-1 1 -1 1 -0.6 1])
pause(1)
figure(3)
scatter3(squeeze(T3(1,4,:)),squeeze(T3(2,4,:)),squeeze(T3(3,4,:)))% �����ͼ
Orochi.plot([0 0 0 0 0 0 0])
axis([-1 1 -1 1 -0.6 1])
pause(1)
scatter3(squeeze(T4(1,4,:)),squeeze(T4(2,4,:)),squeeze(T4(3,4,:)))% �����ͼ
Orochi.plot([0 0 0 0 0 0 0])
axis([-1 1 -1 1 -0.6 1])
pause(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ���ò���
syms q1 q2 q3 q4 q5 q6 q7 Dbs Dse Dew Dwt
global theta1 theta2 theta3 theta4 theta5 theta6 theta7
Dbs = 0; Dse = 0.35; Dew = 0.3; Dwt = 0.1;

%% ��׼DH����
%            theta d     a     alpha  offset
L(1) = Link([0     0     0     -pi/2  0]);
L(2) = Link([0     0     0     pi/2   0]);
L(3) = Link([0     Dse   0     -pi/2  0]);
L(4) = Link([0     0     0     pi/2   0]);
L(5) = Link([0     Dew   0     -pi/2  0]);
L(6) = Link([0     0     0     pi/2   0]);
L(7) = Link([0     Dwt   0     0      0]);
Orochi = SerialLink(L,'name','650mm��Ч�۳�');

i = 10000;
%% ���ؿ��巽����⹤���ռ�
theta1 = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ��һ�ؽڱ�����λ
theta2 = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �ڶ��ؽڱ�����λ
theta3 = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % �����ؽڱ�����λ
theta4 = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % ���Ĺؽڱ�����λ
theta5 = unifrnd(deg2rad(-170),deg2rad(170),[1,i]);    % ����ؽڱ�����λ
theta6 = unifrnd(deg2rad(-120),deg2rad(120),[1,i]);    % �����ؽڱ�����λ
theta7 = unifrnd(deg2rad(-175),deg2rad(175),[1,i]);    % ���߹ؽڱ�����λ 

%% ����Ԫ������
J_00 = cell(i,1); J_01 = cell(i,1); J_02 = cell(i,1); J_03 = cell(i,1);
J_04 = cell(i,1); J_05 = cell(i,1); J_06 = cell(i,1);
J_10 = cell(i,1); J_11 = cell(i,1); J_12 = cell(i,1); J_13 = cell(i,1);
J_14 = cell(i,1); J_15 = cell(i,1); J_16 = cell(i,1);
J_20 = cell(i,1); J_21 = cell(i,1); J_22 = cell(i,1); J_23 = cell(i,1);
J_24 = cell(i,1); J_25 = cell(i,1); J_26 = cell(i,1);
J_30 = cell(i,1); J_31 = cell(i,1); J_32 = cell(i,1); J_33 = cell(i,1);
J_34 = cell(i,1); J_35 = cell(i,1); J_36 = cell(i,1);
J_40 = cell(i,1); J_41 = cell(i,1); J_42 = cell(i,1); J_43 = cell(i,1);
J_44 = cell(i,1); J_45 = cell(i,1); J_46 = cell(i,1);
J_50 = cell(i,1); J_51 = cell(i,1); J_52 = cell(i,1); J_53 = cell(i,1);
J_54 = cell(i,1); J_55 = cell(i,1); J_56 = cell(i,1);

T1 = cell(i,1); 
A = cell(i,1); 

%% ����Jacobian����
for n = 1:i
    q1 = theta1(1,n); q2 = theta2(1,n); q3 = theta3(1,n); q4 = theta4(1,n);
    q5 = theta5(1,n); q6 = theta6(1,n); q7 = theta7(1,n);
    
    J_00{n} = -Dwt*sin(q6)*sin(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) +...
            Dwt*sin(q6)*sin(q4)*sin(q1)*sin(q2)*cos(q5) +...
            Dwt*sin(q6)*sin(q1)*sin(q3)*cos(q2)*sin(q5) - ...
            Dwt*sin(q6)*sin(q3)*cos(q1)*cos(q4)*cos(q5) -...
            Dwt*sin(q4)*sin(q1)*cos(q2)*cos(q3)*cos(q6) -...
            Dew*sin(q4)*sin(q1)*cos(q2)*cos(q3) -...
            Dwt*sin(q6)*cos(q1)*cos(q3)*sin(q5) -...
            Dwt*sin(q4)*sin(q3)*cos(q1)*cos(q6) -...
            Dwt*sin(q1)*cos(q4)*sin(q2)*cos(q6) -...
            Dew*sin(q4)*sin(q3)*cos(q1) -...
            Dew*sin(q1)*cos(q4)*sin(q2) - Dse*sin(q1)*sin(q2);
        
    J_01{n} = -cos(q1)*(Dwt*sin(q6)*cos(q3)*cos(q4)*sin(q2)*cos(q5) +...
            Dwt*sin(q6)*sin(q4)*cos(q2)*cos(q5) -...
            Dwt*sin(q6)*sin(q3)*sin(q2)*sin(q5) +...
            Dwt*sin(q4)*cos(q3)*sin(q2)*cos(q6) +...
            Dew*sin(q4)*cos(q3)*sin(q2) - Dwt*cos(q2)*cos(q4)*cos(q6) -...
            Dew*cos(q2)*cos(q4) - Dse*cos(q2));
        
    J_02{n} = -Dwt*sin(q6)*sin(q3)*cos(q1)*cos(q2)*cos(q4)*cos(q5) -...
            Dwt*sin(q6)*sin(q1)*cos(q3)*cos(q4)*cos(q5) -...
            Dwt*sin(q6)*cos(q1)*cos(q2)*cos(q3)*sin(q5) -...
            Dwt*sin(q4)*sin(q3)*cos(q1)*cos(q2)*cos(q6) -...
            Dew*sin(q4)*sin(q3)*cos(q1)*cos(q2) +...
            Dwt*sin(q6)*sin(q1)*sin(q3)*sin(q5) -...
            Dwt*sin(q4)*sin(q1)*cos(q3)*cos(q6) -...
            Dew*sin(q4)*sin(q1)*cos(q3);
        
    J_03{n} = -Dwt*sin(q6)*sin(q4)*cos(q1)*cos(q2)*cos(q3)*cos(q5) +...
            Dwt*sin(q6)*sin(q4)*sin(q1)*sin(q3)*cos(q5) -...
            Dwt*sin(q6)*cos(q1)*cos(q4)*sin(q2)*cos(q5) +...
            Dwt*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q6) +...
            Dew*cos(q1)*cos(q2)*cos(q3)*cos(q4) -...
            Dwt*sin(q4)*cos(q1)*sin(q2)*cos(q6) -...
            Dwt*sin(q1)*sin(q3)*cos(q4)*cos(q6) -...
            Dew*sin(q4)*cos(q1)*sin(q2) - Dew*sin(q1)*sin(q3)*cos(q4);
        
    J_04{n} = Dwt*sin(q6)*(-cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) +...
            cos(q1)*sin(q2)*sin(q4)*sin(q5) +...
            cos(q4)*sin(q1)*sin(q3)*sin(q5) -...
            cos(q1)*cos(q2)*cos(q5)*sin(q3) -...
            cos(q3)*cos(q5)*sin(q1));
        
    J_05{n} = Dwt*(cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) -...
            cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) -...
            cos(q1)*cos(q5)*cos(q6)*sin(q2)*sin(q4) -...
            cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q3) -...
            cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5) +...
            sin(q1)*sin(q3)*sin(q4)*sin(q6) -...
            cos(q1)*cos(q4)*sin(q2)*sin(q6) -...
            cos(q3)*cos(q6)*sin(q1)*sin(q5));
        
    J_06{n} = 0;
    
    J_10{n} = Dwt*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q6)*cos(q5) +...
            Dwt*cos(q6)*sin(q4)*cos(q1)*cos(q2)*cos(q3) -...
            Dwt*sin(q4)*cos(q1)*sin(q2)*sin(q6)*cos(q5) -...
            Dwt*cos(q1)*sin(q3)*cos(q2)*sin(q6)*sin(q5) -...
            Dwt*sin(q3)*sin(q1)*cos(q4)*sin(q6)*cos(q5) +...
            Dew*sin(q4)*cos(q1)*cos(q2)*cos(q3) -...
            Dwt*cos(q6)*sin(q4)*sin(q3)*sin(q1) +...
            Dwt*cos(q6)*cos(q1)*cos(q4)*sin(q2) -...
            Dwt*cos(q3)*sin(q1)*sin(q6)*sin(q5) -...
            Dew*sin(q4)*sin(q3)*sin(q1) +...
            Dew*cos(q1)*cos(q4)*sin(q2) + Dse*cos(q1)*sin(q2);
        
    J_11{n} = -sin(q1)*(Dwt*cos(q3)*cos(q4)*sin(q2)*sin(q6)*cos(q5) +...
            Dwt*cos(q6)*sin(q4)*cos(q3)*sin(q2) +...
            Dwt*sin(q4)*cos(q2)*sin(q6)*cos(q5) -...
            Dwt*sin(q3)*sin(q2)*sin(q6)*sin(q5) +...
            Dew*sin(q4)*cos(q3)*sin(q2) -...
            Dwt*cos(q6)*cos(q2)*cos(q4) -...
            Dew*cos(q2)*cos(q4) - Dse*cos(q2));
        
    J_12{n} = -Dwt*sin(q3)*cos(q2)*sin(q1)*cos(q4)*sin(q6)*cos(q5) -...
            Dwt*cos(q6)*sin(q4)*sin(q3)*cos(q2)*sin(q1) +...
            Dwt*cos(q1)*cos(q3)*cos(q4)*sin(q6)*cos(q5) -...
            Dwt*cos(q2)*cos(q3)*sin(q1)*sin(q6)*sin(q5) -...
            Dew*sin(q4)*sin(q3)*cos(q2)*sin(q1) +...
            Dwt*cos(q6)*sin(q4)*cos(q1)*cos(q3) -...
            Dwt*cos(q1)*sin(q3)*sin(q6)*sin(q5) +...
            Dew*sin(q4)*cos(q1)*cos(q3);
        
    J_13{n} = -Dwt*cos(q5)*sin(q6)*sin(q4)*cos(q2)*cos(q3)*sin(q1) -...
            Dwt*cos(q5)*sin(q6)*sin(q4)*cos(q1)*sin(q3) -...
            Dwt*cos(q5)*sin(q6)*sin(q1)*cos(q4)*sin(q2) +...
            Dwt*cos(q6)*cos(q2)*cos(q3)*sin(q1)*cos(q4) +...
            Dew*cos(q2)*cos(q3)*sin(q1)*cos(q4) -...
            Dwt*cos(q6)*sin(q4)*sin(q1)*sin(q2) +...
            Dwt*cos(q6)*cos(q1)*sin(q3)*cos(q4) -...
            Dew*sin(q4)*sin(q1)*sin(q2) +...
            Dew*cos(q1)*sin(q3)*cos(q4);
        
    J_14{n} = Dwt*sin(q6)*(-cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) -...
            cos(q2)*cos(q5)*sin(q1)*sin(q3) +...
            sin(q1)*sin(q2)*sin(q4)*sin(q5) -...
            cos(q1)*cos(q4)*sin(q3)*sin(q5) +...
            cos(q1)*cos(q3)*cos(q5));
        
    J_15{n} = -Dwt*(cos(q4)*sin(q1)*sin(q2)*sin(q6) -...
            cos(q1)*cos(q3)*cos(q6)*sin(q5) +...
            cos(q1)*sin(q3)*sin(q4)*sin(q6) -...
            cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q3) +...
            cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) +...
            cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q5) +...
            cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q4) -...
            cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1));
        
    J_16{n} = 0;
    
    J_20{n} = 0;
    
    J_21{n} = -Dwt*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) +...
            Dwt*sin(q5)*cos(q2)*sin(q3)*sin(q6) -...
            Dwt*cos(q2)*sin(q4)*cos(q3)*cos(q6) +...
            Dwt*sin(q4)*sin(q2)*cos(q5)*sin(q6) -...
            Dew*cos(q2)*sin(q4)*cos(q3) -...
            Dwt*cos(q4)*sin(q2)*cos(q6) -...
            Dew*cos(q4)*sin(q2) - Dse*sin(q2);
        
    J_22{n} = sin(q2)*(Dwt*cos(q4)*cos(q5)*sin(q3)*sin(q6) +...
            Dwt*sin(q5)*cos(q3)*sin(q6) +...
            Dwt*sin(q4)*sin(q3)*cos(q6) + Dew*sin(q4)*sin(q3));
        
    J_23{n} = Dwt*sin(q4)*cos(q3)*sin(q2)*cos(q5)*sin(q6) -...
            Dwt*cos(q2)*cos(q4)*cos(q5)*sin(q6) -...
            Dwt*cos(q3)*cos(q4)*sin(q2)*cos(q6) -...
            Dew*cos(q3)*cos(q4)*sin(q2) -...
            Dwt*cos(q2)*sin(q4)*cos(q6) - Dew*cos(q2)*sin(q4);
        
    J_24{n} = Dwt*sin(q6)*(cos(q5)*sin(q2)*sin(q3) +...
            cos(q2)*sin(q4)*sin(q5) +...
            cos(q3)*cos(q4)*sin(q2)*sin(q5));
        
    J_25{n} = Dwt*(-cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) +...
            cos(q3)*sin(q2)*sin(q4)*sin(q6) -...
            cos(q2)*cos(q5)*cos(q6)*sin(q4) +...
            cos(q6)*sin(q2)*sin(q3)*sin(q5) -...
            cos(q2)*cos(q4)*sin(q6));
        
    J_26{n} = 0;
    
    J_30{n} = 0;
    
    J_31{n} = -sin(q1);
    
    J_32{n} = cos(q1)*sin(q2);
    
    J_33{n} = -cos(q1)*cos(q2)*sin(q3) - cos(q3)*sin(q1);
    
    J_34{n} = sin(q4)*cos(q1)*cos(q2)*cos(q3) -...
            sin(q4)*sin(q1)*sin(q3) + cos(q1)*cos(q4)*sin(q2);
        
    J_35{n} = cos(q1)*sin(q2)*sin(q4)*sin(q5) -...
            cos(q1)*cos(q2)*cos(q5)*sin(q3) -...
            cos(q3)*cos(q5)*sin(q1) +...
            cos(q4)*sin(q1)*sin(q3)*sin(q5) -...
            cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5);
        
    J_36{n} = sin(q6)*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) -...
            sin(q6)*sin(q4)*cos(q1)*sin(q2)*cos(q5) -...
            sin(q6)*sin(q1)*sin(q3)*cos(q4)*cos(q5) -...
            sin(q6)*sin(q3)*cos(q1)*cos(q2)*sin(q5) +...
            sin(q4)*cos(q1)*cos(q2)*cos(q3)*cos(q6) -...
            sin(q6)*sin(q1)*cos(q3)*sin(q5) -...
            sin(q4)*sin(q1)*sin(q3)*cos(q6) +...
            cos(q1)*cos(q4)*sin(q2)*cos(q6);
    
    J_40{n} = 0;
    
    J_41{n} = cos(q1);
    
    J_42{n} = sin(q1)*sin(q2);
    
    J_43{n} = cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3);
    
    J_44{n} = sin(q4)*cos(q2)*cos(q3)*sin(q1) +...
            sin(q4)*cos(q1)*sin(q3) + cos(q4)*sin(q1)*sin(q2);
        
    J_45{n} = cos(q1)*cos(q3)*cos(q5) -...
            cos(q2)*cos(q5)*sin(q1)*sin(q3) -...
            cos(q1)*cos(q4)*sin(q3)*sin(q5) +...
            sin(q1)*sin(q2)*sin(q4)*sin(q5) -...
            cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5);
        
    J_46{n} = cos(q3)*cos(q5)*cos(q2)*sin(q1)*cos(q4)*sin(q6) +...
            cos(q1)*cos(q5)*sin(q3)*cos(q4)*sin(q6) +...
            cos(q3)*cos(q2)*sin(q1)*sin(q4)*cos(q6) -...
            cos(q5)*sin(q1)*sin(q2)*sin(q4)*sin(q6) -...
            cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6) +...
            cos(q1)*cos(q3)*sin(q5)*sin(q6) +...
            cos(q1)*sin(q3)*sin(q4)*cos(q6) +...
            sin(q1)*cos(q4)*sin(q2)*cos(q6);
    
    J_50{n} = 1;
    
    J_51{n} = 0;
    
    J_52{n} = cos(q2);
    
    J_53{n} = sin(q2)*sin(q3);
    
    J_54{n} = cos(q2)*cos(q4) - cos(q3)*sin(q2)*sin(q4);
    
    J_55{n} = cos(q5)*sin(q2)*sin(q3) + cos(q2)*sin(q4)*sin(q5) +...
            cos(q3)*cos(q4)*sin(q2)*sin(q5);
        
    J_56{n} = cos(q5)*sin(q2)*cos(q3)*cos(q4)*sin(q6) -...
            cos(q5)*cos(q2)*sin(q4)*sin(q6) +...
            sin(q2)*sin(q3)*sin(q5)*sin(q6) -...
            sin(q2)*sin(q4)*cos(q3)*cos(q6) +...
            cos(q2)*cos(q4)*cos(q6);
end

%% ��Ԫ������ת��Ϊʵ������
J00 = cell2mat(J_00);J01 = cell2mat(J_01);J02 = cell2mat(J_02);
J03 = cell2mat(J_03);J04 = cell2mat(J_04);J05 = cell2mat(J_05);
J06 = cell2mat(J_06);

J10 = cell2mat(J_10);J11 = cell2mat(J_11);J12 = cell2mat(J_12);
J13 = cell2mat(J_13);J14 = cell2mat(J_14);J15 = cell2mat(J_15);
J16 = cell2mat(J_16);

J20 = cell2mat(J_20);J21 = cell2mat(J_21);J22 = cell2mat(J_22);
J23 = cell2mat(J_23);J24 = cell2mat(J_24);J25 = cell2mat(J_25);
J26 = cell2mat(J_26);

J30 = cell2mat(J_30);J31 = cell2mat(J_31);J32 = cell2mat(J_32);
J33 = cell2mat(J_33);J34 = cell2mat(J_34);J35 = cell2mat(J_35);
J36 = cell2mat(J_36);

J40 = cell2mat(J_40);J41 = cell2mat(J_41);J42 = cell2mat(J_42);
J43 = cell2mat(J_43);J44 = cell2mat(J_44);J45 = cell2mat(J_45);
J46 = cell2mat(J_46);

J50 = cell2mat(J_50);J51 = cell2mat(J_51);J52 = cell2mat(J_52);
J53 = cell2mat(J_53);J54 = cell2mat(J_54);J55 = cell2mat(J_55);
J56 = cell2mat(J_56);

for n = 1:i
    T1{n} = [J00(n) J01(n) J03(n) J04(n) J05(n) J06(n);
             J10(n) J11(n) J13(n) J14(n) J15(n) J16(n);
             J20(n) J21(n) J23(n) J24(n) J25(n) J26(n);
             J30(n) J31(n) J33(n) J34(n) J35(n) J36(n);
             J40(n) J41(n) J43(n) J44(n) J45(n) J46(n);
             J50(n) J51(n) J53(n) J54(n) J55(n) J56(n)];
    
    H1 = sqrt(abs(det(cell2mat(T1(n))*(cell2mat(T1(n))'))));
    if H1 < 1e-3
        A{n} = [theta1(1,n) theta2(1,n) theta3(1,n) theta4(1,n)...
                theta5(1,n) theta6(1,n) theta7(1,n)];
    else
        continue
    end
end                                                   

a = cell2mat(A);
T2 = double(Orochi.fkine(a));   % ��е������

%% ��ͼ
figure(5)
c = squeeze(T2(3,4,:));      % c��ʾ��z�������ɫ
scatter3(squeeze(T2(1,4,:)),squeeze(T2(2,4,:)),squeeze(T2(3,4,:)),50,c,'o') 
% scatter3(x,y,z,50,c,'.');  % 50��ʾ��Ĵ�С��c��ʾ��ɫ�����'.'��ʾ�����״
grid on
h = colorbar;                % �Ҳ���ɫ��
set(get(h,'label'),'string','�˶������'); % ���Ҳ���ɫ������

% xlim([22.4931 23.8784]) % X,Y��ȡֵ��Χ
% ylim([112.6833 114.5130])

figure(6)
[m,n]=size(T2);
T3 = zeros(4,4,n/4);
scatter3(squeeze(T2(1,4,:)),squeeze(T3(2,4,:)),squeeze(T2(3,4,:)))% �����ͼ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%