%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人逆运动学求解(2) %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25~2019/12/27
% DH: MDH
% 采用PSO算法对运动学冗余机械臂求一组最优逆解
% 该程序对一个具有7自由度的机械臂做位置控制，由操作空间中的位置坐标，
% 反解出关节空间中的各个关节角度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = Orochi_ikine1(Px,Py,Pz)
global MaxDT t  % MaxDT：为固定迭代次数   MAXDt：为修正最终迭代次数
% DH参数
L(1) = Link([0     0     0     -pi/2  0]);
L(2) = Link([0     0     0     pi/2   0]);
L(3) = Link([0     0.4   0     -pi/2  0]);
L(4) = Link([0     0     0     pi/2   0]);
L(5) = Link([0     0.4   0     -pi/2  0]);
L(6) = Link([0     0     0     pi/2   0]);
L(7) = Link([0     0.1   0     0      0]);
nym_Link = SerialLink(L,'name','nymrobot');
% nym_Link.plot([0 pi/4 -pi/2 0 0 0 pi/2])

% 输入期望位置坐标
% Px = input('Px=');
% Py = input('Py=');
% Pz = input('Pz=');

tic                    % 该函数表示计时开始  
%------初始格式化--------%
% clear;  
% clc;  
format long;  

%------给定初始化条件------%
c1=1.4962;             % 加速常数即学习因子1  
c2=1.4962;             % 加速常数即学习因子2  
w=0.7298;              % 惯性权重  
% w=0.2+0.7*rand;      % 惯性权重
MaxDT=200;             
% 最大迭代次数，迭代次数也可以根据适应度函数值的精度是否满足要求来定 

D=7;                   % 搜索空间维数（一个机械臂的关节变量的个数为7）  
N=70;                  % 群体个体数目  
eps=10^(-7);           % 设置精度(在已知最小值的时候用)  
alpha=10^(-5);

% 机械臂参数（D-H参数）
d1=0.4;    % 长度单位m
d2=0.4;
d3=0.1;

% 定义目标点的空间位置
p_f=[Px,Py,Pz];   

% 机械臂各关节的初始角度
theta1=0;
theta2=pi/4;
theta3=-pi/4;
theta4=0;
theta5=0;
theta6=0;
theta7=pi/2;
q0=[theta1,theta2,theta3,theta4,theta5,theta6,theta7]';

%------初始化种群个体的位置和速度------%  
k=0.5;  % 求取粒子速度系数  vmax=k*xmax  根据各关节的限位范围确定搜索空间

% 各关节的最大限位和最小限位
x_min(1)=-2.967; x_max(1)=2.967;
x_min(2)=-2.094; x_max(2)=2.094;
x_min(3)=-2.967; x_max(3)=2.967;
x_min(4)=-2.094; x_max(4)=2.094;
x_min(5)=-2.967; x_max(5)=2.967;
x_min(6)=-2.094; x_max(6)=2.094;
x_min(7)=-3.054; x_max(7)=3.054;

% 各关节速度的上下限
v_min(1)=x_max(1)*(-k); v_max(1)=x_max(1)*k;
v_min(2)=x_max(1)*(-k); v_max(2)=x_max(1)*k;
v_min(3)=x_max(1)*(-k); v_max(3)=x_max(1)*k;
v_min(4)=x_max(1)*(-k); v_max(4)=x_max(1)*k;
v_min(5)=x_max(1)*(-k); v_max(5)=x_max(1)*k;
v_min(6)=x_max(1)*(-k); v_max(6)=x_max(1)*k;
v_min(7)=x_max(1)*(-k); v_max(7)=x_max(1)*k;

for i=1:N
    % 产生一个服从正态分布的随机数作为初始化位置
    % 产生一个服从正态分布的随机数作为初始化速度
    x(i,1)=rand(1)*(x_min(1)-x_max(1))+x_max(1); 
    v(i,1)=rand(1)*(x_max(1)*(-k)-x_max(1)*k)+x_max(1)*k;  
    x(i,2)=rand(1)*(x_min(2)-x_max(2))+x_max(2);  
    v(i,2)=rand(1)*(x_max(2)*(-k)-x_max(2)*k)+x_max(2)*k;     
    x(i,3)=rand(1)*(x_min(3)-x_max(3))+x_max(3);  
    v(i,3)=rand(1)*(x_max(3)*(-k)-x_max(3)*k)+x_max(3)*k; 
    x(i,4)=rand(1)*(x_min(4)-x_max(4))+x_max(4);  
    v(i,4)=rand(1)*(x_max(4)*(-k)-x_max(4)*k)+x_max(4)*k;
    x(i,5)=rand(1)*(x_min(5)-x_max(5))+x_max(5);  
    v(i,5)=rand(1)*(x_max(5)*(-k)-x_max(5)*k)+x_max(5)*k;     
    x(i,6)=rand(1)*(x_min(6)-x_max(6))+x_max(6);  
    v(i,6)=rand(1)*(x_max(6)*(-k)-x_max(6)*k)+x_max(6)*k; 
    x(i,7)=rand(1)*(x_min(7)-x_max(7))+x_max(7);  
    v(i,7)=rand(1)*(x_max(7)*(-k)-x_max(7)*k)+x_max(7)*k; 
end

% 根据机械臂的正向运动学公式，计算机械臂末端点在操作空间中的位置
for i=1:N
%     p_e(i,1) = d1*cos(x(i,1))*sin(x(i,2)) -...
%                d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
%                cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -....
%                cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
%     p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
%                cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
%                cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
%                d1*sin(x(i,1))*sin(x(i,2));
%     p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
%                cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) + d1*cos(x(i,2));
      p_e(i,1) = d1*cos(x(i,1))*sin(x(i,2)) -...
                 d3*(cos(x(i,6))*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
                 cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
                 cos(x(i,1))*cos(x(i,4))*sin(x(i,2))) +...
                 sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
                 cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) + cos(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
                 sin(x(i,5))*(cos(x(i,3))*sin(x(i,1)) + cos(x(i,1))*cos(x(i,2))*sin(x(i,3))))) -...
                 d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) - cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
                 cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
      p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                 cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
                 cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
                 d3*(cos(x(i,6))*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                 cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
                 cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
                 sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                 cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) -...
                 sin(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
                 sin(x(i,5))*(cos(x(i,1))*cos(x(i,3)) -...
                 cos(x(i,2))*sin(x(i,1))*sin(x(i,3))))) +...
                 d1*sin(x(i,1))*sin(x(i,2));
      p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
                 cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) -...
                 d3*(sin(x(i,6))*(cos(x(i,5))*(cos(x(i,2))*sin(x(i,4)) +...
                 cos(x(i,3))*cos(x(i,4))*sin(x(i,2))) -...
                 sin(x(i,2))*sin(x(i,3))*sin(x(i,5))) -...
                 cos(x(i,6))*(cos(x(i,2))*cos(x(i,4)) -...
                 cos(x(i,3))*sin(x(i,2))*sin(x(i,2)))) + d1*cos(x(i,2));      
end

%------先计算各个粒子的适应度，并初始化个体最优位置y和全局最优位置Pg------%
% 适应度函数是跟上一关节的状态做对比的，该种情况只与初始角度值做比较

% 此时计算p(i)使用的均为初始化的值
for i=1:N  
    % p(i)为适应度函数
    p(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
           (p_f(3)-p_e(i,3))^2)+alpha*((x(i,1)-q0(1))^2+...
           (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
           (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
    y(i,:)=x(i,:);    
    % 初始化个体最优位置y为在时间步t=0时的粒子位置; y()为各个个体的初始化最优位置 
end  

Pg=1;        
% Pg_x为全局最优位置，最优位置包含了7个关节的角度值（初始假想最优值）
Pg_x=x(1,:);    

%%%%%%%%%%%%%%%更新全局最优位置%%%%%%%%%%%%%%%%  
for i=2:N  
    if p(i)<p(Pg)         % 以使适应度函数最小为标准
        Pg=i;   
        Pg_x=x(i,:);      % 初始化 全局最优位置 
    end  
end  

% 创建3维变量空间
record=zeros(N,7,MaxDT);
%------进入主要循环，按照公式依次迭代，直到满足精度要求------------ 
num=1;
for t=1:MaxDT                           % MaxDT为最大迭代次数
    for i=1:N
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(Pg_x-x(i,:));
        %确保关节不超限位范围(限制粒子群的搜索空间)
        if t>1                          % 需要处理第一个位置就超出边界时的情况
            x(i,:)=x(i,:)+v(i,:);       % 判断每次迭代后的位置是否超限制
            for number=1:7
                if x(i,number)>x_max(number) 
                    x(i,number)=record(i,number,t-1);
                end
                if x(i,number)<x_min(number) 
                    x(i,number)=record(i,number,t-1);
                end
            end
            
        else
            x(i,:)=x(i,:)+v(i,:);  
            % 第一代求解完成后不一定都在关节限位范围内（t=1第一代的时候）
            for number=1:7
                if x(i,number)>x_max(number) 
                    x(i,number)=x_max(number);
                end
                if x(i,number)<x_min(number) 
                    x(i,number)=x_min(number);
                end
            end
        end
        
        % 更新末端坐标值
    p_e(i,1) = d1*cos(x(i,1))*sin(x(i,2)) -...
               d3*(cos(x(i,6))*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
               cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
               cos(x(i,1))*cos(x(i,4))*sin(x(i,2))) +...
               sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
               cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) + cos(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
               sin(x(i,5))*(cos(x(i,3))*sin(x(i,1)) + cos(x(i,1))*cos(x(i,2))*sin(x(i,3))))) -...
               d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) - cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
               cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
    p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
               cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
               d3*(cos(x(i,6))*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
               cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
               sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) -...
               sin(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
               sin(x(i,5))*(cos(x(i,1))*cos(x(i,3)) -...
               cos(x(i,2))*sin(x(i,1))*sin(x(i,3))))) +...
               d1*sin(x(i,1))*sin(x(i,2));
    p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
               cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) -...
               d3*(sin(x(i,6))*(cos(x(i,5))*(cos(x(i,2))*sin(x(i,4)) +...
               cos(x(i,3))*cos(x(i,4))*sin(x(i,2))) -...
               sin(x(i,2))*sin(x(i,3))*sin(x(i,5))) -...
               cos(x(i,6))*(cos(x(i,2))*cos(x(i,4)) -...
               cos(x(i,3))*sin(x(i,2))*sin(x(i,2)))) + d1*cos(x(i,2));   
        
        % 更新后重新计算适应度函数
    fit(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
             (p_f(3)-p_e(i,3))^2) + alpha*((x(i,1)-q0(1))^2+...
             (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
             (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
        
    if fit(i)<p(i)
        p(i)=fit(i);      % 更新适应度
        y(i,:)=x(i,:);    % 更新个体最佳位置
    end
          
        % 更新Pg处的末端坐标值
    p_e(Pg,1) = d1*cos(x(i,1))*sin(x(i,2)) -...
                d3*(cos(x(i,6))*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
                cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
                cos(x(i,1))*cos(x(i,4))*sin(x(i,2))) +...
                sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
                cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) + cos(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
                sin(x(i,5))*(cos(x(i,3))*sin(x(i,1)) + cos(x(i,1))*cos(x(i,2))*sin(x(i,3))))) -...
                d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) - cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
                cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
    p_e(Pg,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
                cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
                d3*(cos(x(i,6))*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
                cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
                sin(x(i,6))*(cos(x(i,5))*(cos(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) -...
                sin(x(i,1))*sin(x(i,2))*sin(x(i,4))) +...
                sin(x(i,5))*(cos(x(i,1))*cos(x(i,3)) -...
                cos(x(i,2))*sin(x(i,1))*sin(x(i,3))))) +...
                d1*sin(x(i,1))*sin(x(i,2)); 
    p_e(Pg,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
                cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) -...
                d3*(sin(x(i,6))*(cos(x(i,5))*(cos(x(i,2))*sin(x(i,4)) +...
                cos(x(i,3))*cos(x(i,4))*sin(x(i,2))) -...
                sin(x(i,2))*sin(x(i,3))*sin(x(i,5))) -...
                cos(x(i,6))*(cos(x(i,2))*cos(x(i,4)) -...
                cos(x(i,3))*sin(x(i,2))*sin(x(i,2)))) + d1*cos(x(i,2));   
  
    fit(Pg) = sqrt((p_f(1)-p_e(Pg,1))^2+(p_f(2)-p_e(Pg,2))^2+...
              (p_f(3)-p_e(Pg,3))^2)+alpha*((x(Pg,1)-q0(1))^2+...
              (x(Pg,2)-q0(2))^2+(x(Pg,3)-q0(3))^2+(x(Pg,4)-q0(4))^2+...
              (x(Pg,5)-q0(5))^2+(x(Pg,6)-q0(6))^2+(x(Pg,7)-q0(7))^2);
        
     if p(i)<fit(Pg)
         Pg=i;
         Pg_x=y(i,:);% 更新群体最佳位置
     end
    end   
    record(:,:,t)=x(:,:);                % 记录每一次学习过程中的中间位置值
    %%%至此学习完了1代%%%%
    Pbest(t)=fit(Pg);                    % 保存每一代的群体最佳位置
    P_1(t,:) = Pg_x(1,1);
    P_2(t,:) = Pg_x(1,2);
    P_3(t,:) = Pg_x(1,3);
    P_4(t,:) = Pg_x(1,4);
    P_5(t,:) = Pg_x(1,5);
    P_6(t,:) = Pg_x(1,6);
    P_7(t,:) = Pg_x(1,7);
end

% 输出最后的计算结果
disp('函数的全局最优位置为：')  

for i=1:D  
    fprintf('x(%d)=%s  ',i,Pg_x(i));
end  

% 验证最后求得的最优解中各个关节角度是否均在限位以内
if Pg_x(1)>=x_min(1) && Pg_x(1)<=x_max(1) && Pg_x(2)>=x_min(2) &&...
   Pg_x(2)<=x_max(2) && Pg_x(3)>=x_min(3) && Pg_x(3)<=x_max(3) &&...
   Pg_x(4)>=x_min(4) && Pg_x(4)<=x_max(4) && Pg_x(5)>=x_min(5) &&...
   Pg_x(5)<=x_max(5) && Pg_x(6)>=x_min(6) && Pg_x(6)<=x_max(6) &&...
   Pg_x(7)>=x_min(7) && Pg_x(7)<=x_max(7) 

    fprintf('\n');
    fprintf('Right!\n');
else
    fprintf('Error!\n')
end

fprintf('最后得到的优化极值为：%s\n',fit(Pg));   %应该观察fit(Pg)的数量级
fprintf('迭代次数：%d\n',MaxDT);

p_e(Pg,:)
toc  % 该函数表示计时结束

figure(1)
nym_Link.plot([Pg_x(1) Pg_x(2) Pg_x(3) Pg_x(4) Pg_x(5) Pg_x(6) Pg_x(7)])

figure(2)
plot3(p_e(Pg,1),p_e(Pg,2),p_e(Pg,3),'bo')
hold on 
plot3(Px,Py,Pz,'r*')
grid on

% T = double(nymfkine(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7)));                % 机械臂正解
J = Orochi_jacob(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7));
Rd = rank(J);         % 雅克比行列式的秩
D = sqrt(det(J*J'));  % 可操作度
if Rd==6 && D > 1e-3
    fprintf('该组解为可操作解,该组雅克比行列式的秩=6\n')
    fprintf('该点处的可操作度D = %s\n',D)
else
    Point = Orochi_fkine(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7));
    fprintf('该组解为不可操作解,该组雅克比行列式的秩<6\n')
    fprintf('该点为运动学奇异点：%s %s %s\n',Point(1,4),Point(2,4),Point(3,4))
    fprintf('该点处的可操作度D = 0\n')
end
% figure(2)
% plot(MaxDT,p_e(Pg,1),'ro-',MaxDT,p_e(Pg,2),'bo-',MaxDT,p_e(Pg,3),'go-')

%%%%%%%%%%%% 可操作度图像 %%%%%%%%%%%%

t=1:MaxDT;
figure(3)
plot(t,P_1(:,1),'b-')
grid on

figure(4)
plot(t,P_2(:,1),'b-')
grid on

figure(5)
plot(t,P_3(:,1),'b-')
grid on

figure(6)
plot(t,P_4(:,1),'b-')
grid on

figure(7)
plot(t,P_5(:,1),'b-')
grid on

figure(8)
plot(t,P_6(:,1),'b-')
grid on

figure(9)
plot(t,P_7(:,1),'b-')
grid on
end













