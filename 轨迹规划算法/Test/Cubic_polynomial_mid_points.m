%%%%%%%%%%%%%%%%%%%%% ���м������ζ���ʽ��ֵ���߲��� %%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/20
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
X = [0 2 4 6 8 10];
V = [10 18 35 21 15 25];
Xq=0:0.01:X(end);
n=length(Xq);
for i=1:n
    Vq(i)=interp1(X,V,Xq(i),'pchip');
end
figure('Name','���м�����ζ���ʽ��ֵ����')
plot(X,V,'ro','MarkerFaceColor','r')  % ·����
hold on
plot(Xq,Vq)
grid on
xlabel('Time(s)');ylabel('��(deg)');
xlim([0 X(end)]);
title('���м�����ζ���ʽ��ֵ����');