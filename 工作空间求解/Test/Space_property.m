%%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人工作空间曲线 %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\x_1.mat')
load('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\file_mat\y_S.mat')
l = 0.1:0.05:0.8;
S = [0.6084 0.6412 0.7313 0.7643 0.8276 0.8828 0.9012...
     0.9351 0.9523 0.9529 0.9272 0.8506 0.7848 0.7034 0.6310];
fprintf('工作空间最大值坐标(0.5 0.9529)\n') 
p=polyfit(l,S,3);
y1=polyval(p,l);
ymax = max(y1);
plot(l,y1)
hold on
plot(0.5,ymax,'r*')
xlabel('总臂长为1m时的大臂长度(肩肘距离)/m'); ylabel('归一化后的工作空间指标')
grid on
text(0.5,0.9417,'工作空间最大值(0.5 0.9417)')
legend('工作空间曲线','工作空间最大值')
axis([0.05,0.85 0.55,1])
hold on