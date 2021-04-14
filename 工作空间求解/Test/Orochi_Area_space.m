%%%%%%%%%%%%%%%%%%%%%%%%%% 工作空间截面图灰度处理 %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/18
% DH: SDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
clc
files = dir(fullfile('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\workspace_picture\','*.bmp'));
Output_path='C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\rgb_picture\';
lengthFiles = length(files);
for i = 1:lengthFiles
    Img = imread(strcat('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\workspace_picture\',files(i).name));%文件所在路径
    disp(strcat('C:\Users\18292\Documents\MATLAB\Orochi for MATLAB(镁伽项目)\工作空间求解\Test\workspace_picture\',files(i).name)) %打印文件路径
    figure(2*i-1)
    imshow(Img)
    figure(2*i)
    BW1=rgb2gray(Img);
    imshow(BW1)
    imwrite(BW1,[Output_path,'Img_out',num2str(i),'.bmp']);
    [m,n]=size(BW1);
    for k=1:m
        for j=1:n
            if(BW1(k,j)>=100)
                BW1(k,j)=0;
            else
                BW1(k,j)=1;
            end
        end
    end
    total(i)=bwarea(BW1)/(5.5e6);
end

figure(31)
l = 0.1:0.05:0.8;
S = [0.6084 0.6412 0.7313 0.7643 0.8276 0.8828 0.9012...
     0.9351 0.9523 0.9529 0.9272 0.8506 0.7848 0.7034 0.6310];
fprintf('最大值坐标(0.5 0.9529)\n') 
p=polyfit(l,S,3);
y1=polyval(p,l);
ymax = max(y1);
plot(l,y1)
hold on
plot(0.5,ymax,'r*')
xlabel('总臂长/m'); ylabel('归一化后的工作空间指标')
grid on
text(0.5,0.9417,'最大值坐标(0.5 0.9417)')
legend('工作空间值','最大值坐标')
axis([0.05,0.85 0.55,1])
