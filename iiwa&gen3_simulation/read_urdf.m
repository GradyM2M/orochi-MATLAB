%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% URDFģ�Ͷ�ȡ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/13
% ͨ��importrobot�������ɶ�ȡSolidworks������URDFģ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
figure(1)
subplot(1,2,1)
robot = importrobot('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\iiwa&gen3_simulation\Orochi.urdf');
show(robot)
axis([-1 1 -1 1 0 2])

subplot(1,2,2)
robot = importrobot('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(þ٤��Ŀ)\orochi\urdf\orochi.urdf');
show(robot)
axis([-1 1 -1 1 0 2])