%%%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�������˿ɲ��������� %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\x_1.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\y_1.mat')
ymax=max(y); % �󼫴�ֵ
figure(1)
fprintf('�ɲ��������ֵ����(0.7806 0.9568)\n');
plot(x,y)
xlabel('�ܱ۳�/m');ylabel('��һ����Ŀɲ�����ָ��')
grid on
hold on
plot(0.7806,ymax,'r*')
text(0.7806,ymax,'�ɲ��������ֵ(0.7806 0.9568)')
legend('�ɲ���������','�ɲ��������ֵ');
hold on

% text(1,0.65,'��������Ϊ0.785~0.835֮��')