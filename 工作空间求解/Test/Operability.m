%%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人可操作度曲线 %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\x_1.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\y_1.mat')
ymax=max(y); % 求极大值
figure(1)
fprintf('可操作度最大值坐标(0.7806 0.9568)\n');
plot(x,y)
xlabel('总臂长/m');ylabel('归一化后的可操作度指标')
grid on
hold on
plot(0.7806,ymax,'r*')
text(0.7806,ymax,'可操作度最大值(0.7806 0.9568)')
legend('可操作度曲线','可操作度最大值');
hold on

% text(1,0.65,'最优区间为0.785~0.835之间')