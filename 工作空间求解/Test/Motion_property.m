%%%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ���������˶��������� %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\x_2.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\�����ռ����\Test\file_mat\y_2.mat')
ymax=max(stiffn); % �󼫴�ֵ
fprintf('�˶��������ֵ����(0.8388 0.9294)\n');
plot(x, stiffn,'r')
hold on
plot(0.8388,ymax,'g*')
xlabel('�ܱ۳�/m'); ylabel('��һ������˶�����ָ��')
grid on
text(0.8388,ymax,'�˶��������ֵ(0.8388 0.9294)')
legend('�˶���������','�˶��������ֵ');
hold on
