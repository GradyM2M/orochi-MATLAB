%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% URDF模型读取 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/13
% 通过importrobot函数即可读取Solidworks导出的URDF模型
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
figure(1)
subplot(1,2,1)
robot = importrobot('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\iiwa&gen3_simulation\Orochi.urdf');
show(robot)
axis([-1 1 -1 1 0 2])

subplot(1,2,2)
robot = importrobot('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\orochi\urdf\orochi.urdf');
show(robot)
axis([-1 1 -1 1 0 2])