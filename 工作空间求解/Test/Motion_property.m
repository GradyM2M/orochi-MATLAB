%%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人运动性能曲线 %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\x_2.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\y_2.mat')
ymax=max(stiffn); % 求极大值
fprintf('运动性能最大值坐标(0.8388 0.9294)\n');
plot(x, stiffn,'r')
hold on
plot(0.8388,ymax,'g*')
xlabel('总臂长/m'); ylabel('归一化后的运动性能指标')
grid on
text(0.8388,ymax,'运动性能最大值(0.8388 0.9294)')
legend('运动性能曲线','运动性能最大值');
hold on
