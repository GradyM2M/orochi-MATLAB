%%%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�������˹����ռ����� %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\x_1.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\y_S.mat')
l = 0.1:0.05:0.8;
S = [0.6084 0.6412 0.7313 0.7643 0.8276 0.8828 0.9012...
     0.9351 0.9523 0.9529 0.9272 0.8506 0.7848 0.7034 0.6310];
fprintf('�����ռ����ֵ����(0.5 0.9529)\n') 
p=polyfit(l,S,3);
y1=polyval(p,l);
ymax = max(y1);
plot(l,y1)
hold on
plot(0.5,ymax,'r*')
xlabel('�ܱ۳�Ϊ1mʱ�Ĵ�۳���(�������)/m'); ylabel('��һ����Ĺ����ռ�ָ��')
grid on
text(0.5,0.9417,'�����ռ����ֵ(0.5 0.9417)')
legend('�����ռ�����','�����ռ����ֵ')
axis([0.05,0.85 0.55,1])
hold on