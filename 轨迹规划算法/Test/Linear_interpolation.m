%%%%%%%%%%%%%%%%%%%%%%%%%%%% ֱ�߹켣�岹���� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/26
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

% ��ʼ�㡢��ֹ�㡢����滮ʱ��
P0 = [-0.3 0.6 -0.1 pi 0 pi/3];
Pf = [0.2 0.4 0 pi 0 0];
ts = 5;
t = 0:0.01:ts;
l = [0.475 0.325];
%%��ʼλ��P0
RPY0 = P0(4:6);                  % P0��RPY��
T0=transl(P0(1:3))*trotz(RPY0(3))*troty(RPY0(2))*trotx(RPY0(1));   % P0��λ�˾���

%��ֹλ��Pf
RPYf=Pf(4:6);
Tf=transl(Pf(1:3))*trotz(RPYf(3))*troty(RPYf(2))*trotx(RPYf(1));

%�岹
N = length(t);
% λ��ֱ�߲岹
P(:,1) = P0(1:3)';
dP = (Pf(1:3)'-P0(1:3)')/(N-1);
for i=2:N
    P(:,i) = P0(1:3)'+(i-1)*dP;
end
f = RPYf-RPY0;
% RPY�����ζ���ʽ�岹
for i=1:N
    RPY(i,:) = RPY0+(3*t(i)^2/ts^2-2*t(i)^3/ts^3)*f; 
end

%%��ͼ
figure('Name','������ĩ��ֱ���˶��켣');
plot3(P0(1),P0(2),P0(3),'ro','MarkerFaceColor','r')

hold on
plot3(Pf(1),Pf(2),Pf(3),'rv','MarkerFaceColor','b')

hold on
x=P(1,:);
y=P(2,:);
z=P(3,:);
plot3(x,y,z,'-');
legend('��ʼλ��','��ֹλ��','ֱ�߹켣')
grid on
xlabel('X'); ylabel('Y'); zlabel('Z')

figure('Name','�����˹ؽ�λ������');
subplot(3,1,1)
plot(t,x);
grid on
xlim([0,t(end)]);
xlabel('ʱ��(s)'); ylabel('Px');
subplot(3,1,2)
plot(t,y);
grid on
xlim([0,t(end)]);
xlabel('ʱ��(s)'); ylabel('Py');
subplot(3,1,3)
plot(t,z);
grid on
xlim([0,t(end)]);
xlabel('ʱ��(s)'); ylabel('Pz');