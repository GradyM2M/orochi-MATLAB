%%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�����������˶�ѧ���(2) %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25~2019/12/27
% DH: MDH
% ����PSO�㷨���˶�ѧ�����е����һ���������
% �ó����һ������7���ɶȵĻ�е����λ�ÿ��ƣ��ɲ����ռ��е�λ�����꣬
% ������ؽڿռ��еĸ����ؽڽǶ�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = Orochi_ikine1(Px,Py,Pz)
global MaxDT t  % MaxDT��Ϊ�̶���������   MAXDt��Ϊ�������յ�������
% DH����
L(1) = Link([0     0     0     -pi/2  0]);
L(2) = Link([0     0     0     pi/2   0]);
L(3) = Link([0     0.4   0     -pi/2  0]);
L(4) = Link([0     0     0     pi/2   0]);
L(5) = Link([0     0.4   0     -pi/2  0]);
L(6) = Link([0     0     0     pi/2   0]);
L(7) = Link([0     0.1   0     0      0]);
nym_Link = SerialLink(L,'name','nymrobot');
% nym_Link.plot([0 pi/4 -pi/2 0 0 0 pi/2])

% ��������λ������
% Px = input('Px=');
% Py = input('Py=');
% Pz = input('Pz=');

tic                    % �ú�����ʾ��ʱ��ʼ  
%------��ʼ��ʽ��--------%
% clear;  
% clc;  
format long;  

%------������ʼ������------%
c1=1.4962;             % ���ٳ�����ѧϰ����1  
c2=1.4962;             % ���ٳ�����ѧϰ����2  
w=0.7298;              % ����Ȩ��  
% w=0.2+0.7*rand;      % ����Ȩ��
MaxDT=200;             
% ��������������������Ҳ���Ը�����Ӧ�Ⱥ���ֵ�ľ����Ƿ�����Ҫ������ 

D=7;                   % �����ռ�ά����һ����е�۵Ĺؽڱ����ĸ���Ϊ7��  
N=70;                  % Ⱥ�������Ŀ  
eps=10^(-7);           % ���þ���(����֪��Сֵ��ʱ����)  
alpha=10^(-5);

% ��е�۲�����D-H������
d1=0.4;    % ���ȵ�λm
d2=0.4;
d3=0.1;

% ����Ŀ���Ŀռ�λ��
p_f=[Px,Py,Pz];   

% ��е�۸��ؽڵĳ�ʼ�Ƕ�
theta1=0;
theta2=pi/4;
theta3=-pi/4;
theta4=0;
theta5=0;
theta6=0;
theta7=pi/2;
q0=[theta1,theta2,theta3,theta4,theta5,theta6,theta7]';

%------��ʼ����Ⱥ�����λ�ú��ٶ�------%  
k=0.5;  % ��ȡ�����ٶ�ϵ��  vmax=k*xmax  ���ݸ��ؽڵ���λ��Χȷ�������ռ�

% ���ؽڵ������λ����С��λ
x_min(1)=-2.967; x_max(1)=2.967;
x_min(2)=-2.094; x_max(2)=2.094;
x_min(3)=-2.967; x_max(3)=2.967;
x_min(4)=-2.094; x_max(4)=2.094;
x_min(5)=-2.967; x_max(5)=2.967;
x_min(6)=-2.094; x_max(6)=2.094;
x_min(7)=-3.054; x_max(7)=3.054;

% ���ؽ��ٶȵ�������
v_min(1)=x_max(1)*(-k); v_max(1)=x_max(1)*k;
v_min(2)=x_max(1)*(-k); v_max(2)=x_max(1)*k;
v_min(3)=x_max(1)*(-k); v_max(3)=x_max(1)*k;
v_min(4)=x_max(1)*(-k); v_max(4)=x_max(1)*k;
v_min(5)=x_max(1)*(-k); v_max(5)=x_max(1)*k;
v_min(6)=x_max(1)*(-k); v_max(6)=x_max(1)*k;
v_min(7)=x_max(1)*(-k); v_max(7)=x_max(1)*k;

for i=1:N
    % ����һ��������̬�ֲ����������Ϊ��ʼ��λ��
    % ����һ��������̬�ֲ����������Ϊ��ʼ���ٶ�
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

% ���ݻ�е�۵������˶�ѧ��ʽ�������е��ĩ�˵��ڲ����ռ��е�λ��
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

%------�ȼ���������ӵ���Ӧ�ȣ�����ʼ����������λ��y��ȫ������λ��Pg------%
% ��Ӧ�Ⱥ����Ǹ���һ�ؽڵ�״̬���Աȵģ��������ֻ���ʼ�Ƕ�ֵ���Ƚ�

% ��ʱ����p(i)ʹ�õľ�Ϊ��ʼ����ֵ
for i=1:N  
    % p(i)Ϊ��Ӧ�Ⱥ���
    p(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
           (p_f(3)-p_e(i,3))^2)+alpha*((x(i,1)-q0(1))^2+...
           (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
           (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
    y(i,:)=x(i,:);    
    % ��ʼ����������λ��yΪ��ʱ�䲽t=0ʱ������λ��; y()Ϊ��������ĳ�ʼ������λ�� 
end  

Pg=1;        
% Pg_xΪȫ������λ�ã�����λ�ð�����7���ؽڵĽǶ�ֵ����ʼ��������ֵ��
Pg_x=x(1,:);    

%%%%%%%%%%%%%%%����ȫ������λ��%%%%%%%%%%%%%%%%  
for i=2:N  
    if p(i)<p(Pg)         % ��ʹ��Ӧ�Ⱥ�����СΪ��׼
        Pg=i;   
        Pg_x=x(i,:);      % ��ʼ�� ȫ������λ�� 
    end  
end  

% ����3ά�����ռ�
record=zeros(N,7,MaxDT);
%------������Ҫѭ�������չ�ʽ���ε�����ֱ�����㾫��Ҫ��------------ 
num=1;
for t=1:MaxDT                           % MaxDTΪ����������
    for i=1:N
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(Pg_x-x(i,:));
        %ȷ���ؽڲ�����λ��Χ(��������Ⱥ�������ռ�)
        if t>1                          % ��Ҫ�����һ��λ�þͳ����߽�ʱ�����
            x(i,:)=x(i,:)+v(i,:);       % �ж�ÿ�ε������λ���Ƿ�����
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
            % ��һ�������ɺ�һ�����ڹؽ���λ��Χ�ڣ�t=1��һ����ʱ��
            for number=1:7
                if x(i,number)>x_max(number) 
                    x(i,number)=x_max(number);
                end
                if x(i,number)<x_min(number) 
                    x(i,number)=x_min(number);
                end
            end
        end
        
        % ����ĩ������ֵ
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
        
        % ���º����¼�����Ӧ�Ⱥ���
    fit(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
             (p_f(3)-p_e(i,3))^2) + alpha*((x(i,1)-q0(1))^2+...
             (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
             (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
        
    if fit(i)<p(i)
        p(i)=fit(i);      % ������Ӧ��
        y(i,:)=x(i,:);    % ���¸������λ��
    end
          
        % ����Pg����ĩ������ֵ
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
         Pg_x=y(i,:);% ����Ⱥ�����λ��
     end
    end   
    record(:,:,t)=x(:,:);                % ��¼ÿһ��ѧϰ�����е��м�λ��ֵ
    %%%����ѧϰ����1��%%%%
    Pbest(t)=fit(Pg);                    % ����ÿһ����Ⱥ�����λ��
    P_1(t,:) = Pg_x(1,1);
    P_2(t,:) = Pg_x(1,2);
    P_3(t,:) = Pg_x(1,3);
    P_4(t,:) = Pg_x(1,4);
    P_5(t,:) = Pg_x(1,5);
    P_6(t,:) = Pg_x(1,6);
    P_7(t,:) = Pg_x(1,7);
end

% ������ļ�����
disp('������ȫ������λ��Ϊ��')  

for i=1:D  
    fprintf('x(%d)=%s  ',i,Pg_x(i));
end  

% ��֤�����õ����Ž��и����ؽڽǶ��Ƿ������λ����
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

fprintf('���õ����Ż���ֵΪ��%s\n',fit(Pg));   %Ӧ�ù۲�fit(Pg)��������
fprintf('����������%d\n',MaxDT);

p_e(Pg,:)
toc  % �ú�����ʾ��ʱ����

figure(1)
nym_Link.plot([Pg_x(1) Pg_x(2) Pg_x(3) Pg_x(4) Pg_x(5) Pg_x(6) Pg_x(7)])

figure(2)
plot3(p_e(Pg,1),p_e(Pg,2),p_e(Pg,3),'bo')
hold on 
plot3(Px,Py,Pz,'r*')
grid on

% T = double(nymfkine(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7)));                % ��е������
J = Orochi_jacob(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7));
Rd = rank(J);         % �ſ˱�����ʽ����
D = sqrt(det(J*J'));  % �ɲ�����
if Rd==6 && D > 1e-3
    fprintf('�����Ϊ�ɲ�����,�����ſ˱�����ʽ����=6\n')
    fprintf('�õ㴦�Ŀɲ�����D = %s\n',D)
else
    Point = Orochi_fkine(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7));
    fprintf('�����Ϊ���ɲ�����,�����ſ˱�����ʽ����<6\n')
    fprintf('�õ�Ϊ�˶�ѧ����㣺%s %s %s\n',Point(1,4),Point(2,4),Point(3,4))
    fprintf('�õ㴦�Ŀɲ�����D = 0\n')
end
% figure(2)
% plot(MaxDT,p_e(Pg,1),'ro-',MaxDT,p_e(Pg,2),'bo-',MaxDT,p_e(Pg,3),'go-')

%%%%%%%%%%%% �ɲ�����ͼ�� %%%%%%%%%%%%

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













