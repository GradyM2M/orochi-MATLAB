%%%%%%%%%%%%%�Ľ�PSO�㷨���˶�ѧ�����е����һ���������(Test 2)%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/1/05~2020/1/07
% �ó����һ������7���ɶȵĻ�е����λ�ÿ��ƣ��ɲ����ռ��е�λ�����꣬
% ������ؽڿռ��еĸ����ؽڽǶȣ�
% ʹ��һ��������·���켣���в��ԣ�ѡȡ�����Ƕ�Ϊ5��

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
close all
global z MaxDT MAXDt t   % MaxDT��Ϊ�̶���������   MAXDt��Ϊ�������յ�������
global P_1 P_2 P_3 P_4 P_5 P_6 P_7 N
% DH����
L(1) = Link([0     0     0     -pi/2  0]);
L(2) = Link([0     0     0     pi/2   0]);
L(3) = Link([0     0.5   0     -pi/2  0]);
L(4) = Link([0     0     0     pi/2   0]);
L(5) = Link([0     0.4   0     -pi/2  0]);
L(6) = Link([0     0     0     pi/2   0]);
L(7) = Link([0     0     0     0      0]);
nym_Link = SerialLink(L,'name','nymrobot');
% teach(nym_Link)
Q = cell(73,4);

tic                    % �ú�����ʾ��ʱ��ʼ  
for j=1:1:73
%--------��ʼ��ʽ��--------%
% clear;  
% clc;  
format long;  

%------������ʼ������------%
c1=1.4962;             % ���ٳ�����ѧϰ����1  
c2=1.4962;             % ���ٳ�����ѧϰ����2  
w=0.7298;              % ����Ȩ��  
% w=0.2+0.7*rand;      % ����Ȩ��
MaxDT = 200;             
% ��������������������Ҳ���Ը�����Ӧ�Ⱥ���ֵ�ľ����Ƿ�����Ҫ������ 

D=7;                   % �����ռ�ά����һ����е�۵Ĺؽڱ����ĸ���Ϊ7��  
N=70;                  % Ⱥ�������Ŀ  
eps=10^(-7);           % ���þ���(����֪��Сֵ��ʱ����)  
alpha=10^(-5);

% ��е�۲�����D-H������
d1=0.5;    % ���ȵ�λm
d2=0.4;

% ����Ŀ���Ŀռ�λ��
Px = 0.2*sin(pi*(j-1)/36); 
Py = 0.2*cos(pi*(j-1)/36); 
Pz = 0.5+(j-1)/720;
p_f = [Px,Py,Pz];   

% ��е�۸��ؽڵĳ�ʼ�Ƕ�
theta1 = 0;
theta2 = pi/4;
theta3 = -pi/2;
theta4 = 0;
theta5 = 0;
theta6 = 0;
theta7 = pi/2;
q0=[theta1,theta2,theta3,theta4,theta5,theta6,theta7]';

%------��ʼ����Ⱥ�����λ�ú��ٶ�------%  
k = 0.5;  % ��ȡ�����ٶ�ϵ��  vmax=k*xmax  ���ݸ��ؽڵ���λ��Χȷ�������ռ�

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

for i = 1:N
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
for i = 1:N
    p_e(i,1) = d1*cos(x(i,1))*sin(x(i,2)) -...
               d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
               cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -....
               cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
    p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
               cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
               d1*sin(x(i,1))*sin(x(i,2));
    p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
               cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) + d1*cos(x(i,2));
end

%------�ȼ���������ӵ���Ӧ�ȣ�����ʼ����������λ��y��ȫ������λ��Pg------%
% ��Ӧ�Ⱥ����Ǹ���һ�ؽڵ�״̬���Աȵģ��������ֻ���ʼ�Ƕ�ֵ���Ƚ�

% ��ʱ����p(i)ʹ�õľ�Ϊ��ʼ����ֵ
for i = 1:N  
    % p(i)Ϊ��Ӧ�Ⱥ���
    p(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
           (p_f(3)-p_e(i,3))^2)+alpha*((x(i,1)-q0(1))^2+...
           (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
           (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
    y(i,:) = x(i,:);    
    % ��ʼ����������λ��yΪ��ʱ�䲽t=0ʱ������λ��; y()Ϊ��������ĳ�ʼ������λ�� 
end  

Pg = 1;        
% Pg_xΪȫ������λ�ã�����λ�ð�����7���ؽڵĽǶ�ֵ����ʼ��������ֵ��

Pg_x = x(1,:);    

%%%%%%%%%%%%%%%����ȫ������λ��%%%%%%%%%%%%%%%%  
for i = 2:N  
    if p(i)<p(Pg)         % ��ʹ��Ӧ�Ⱥ�����СΪ��׼
        Pg=i;   
        Pg_x=x(i,:);      % ��ʼ�� ȫ������λ�� 
    end  
end  

% ����3ά�����ռ�
record=zeros(N,7,MaxDT);
%------������Ҫѭ�������չ�ʽ���ε�����ֱ�����㾫��Ҫ��------%
num=1;
for t = 1:MaxDT                         % MaxDTΪ����������
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
               d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
               cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
               cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
    p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
               cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
               d1*sin(x(i,1))*sin(x(i,2));
    p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
               cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) + d1*cos(x(i,2));
        
        % ���º����¼�����Ӧ�Ⱥ���
    fit(i) = sqrt((p_f(1)-p_e(i,1))^2+(p_f(2)-p_e(i,2))^2+...
             (p_f(3)-p_e(i,3))^2) + alpha*((x(i,1)-q0(1))^2+...
             (x(i,2)-q0(2))^2+(x(i,3)-q0(3))^2+(x(i,4)-q0(4))^2+...
             (x(i,5)-q0(5))^2+(x(i,6)-q0(6))^2+(x(i,7)-q0(7))^2);
        
    if fit(i)<p(i)
        p(i) = fit(i);      % ������Ӧ��
        y(i,:) = x(i,:);    % ���¸������λ��
    end
          
        % ����Pg����ĩ������ֵ
    p_e(Pg,1) = d1*cos(x(Pg,1))*sin(x(Pg,2)) -... 
                d2*(sin(x(Pg,4))*(sin(x(Pg,1))*sin(x(Pg,3)) -...
                cos(x(Pg,1))*cos(x(Pg,2))*cos(x(Pg,3))) -... 
                cos(x(Pg,1))*cos(x(Pg,4))*sin(x(Pg,2)));
    p_e(Pg,2) = d2*(sin(x(Pg,4))*(cos(x(Pg,1))*sin(x(Pg,3)) +...
                cos(x(Pg,2))*cos(x(Pg,3))*sin(x(Pg,1))) +...
                cos(x(Pg,4))*sin(x(Pg,1))*sin(x(Pg,2))) +...
                d1*sin(x(Pg,1))*sin(x(Pg,2)); 
    p_e(Pg,3) = d2*(cos(x(Pg,2))*cos(x(Pg,4)) -...
                cos(x(Pg,3))*sin(x(Pg,2))*sin(x(Pg,4))) + d1*cos(x(Pg,2));
  
    fit(Pg) = sqrt((p_f(1)-p_e(Pg,1))^2+(p_f(2)-p_e(Pg,2))^2+...
              (p_f(3)-p_e(Pg,3))^2)+alpha*((x(Pg,1)-q0(1))^2+...
              (x(Pg,2)-q0(2))^2+(x(Pg,3)-q0(3))^2+(x(Pg,4)-q0(4))^2+...
              (x(Pg,5)-q0(5))^2+(x(Pg,6)-q0(6))^2+(x(Pg,7)-q0(7))^2);
        
     if p(i)<fit(Pg)
         Pg = i;
         Pg_x = y(i,:);% ����Ⱥ�����λ��
     end
    end   
    record(:,:,t)=x(:,:);                % ��¼ÿһ��ѧϰ�����е��м�λ��ֵ
    %%%����ѧϰ����1��%%%%
    Pbest(t)=fit(Pg);                    % ����ÿһ����Ⱥ�����λ��
    
    P_1(t,j,:) = Pg_x(1,1);
    P_2(t,j,:) = Pg_x(1,2);
    P_3(t,j,:) = Pg_x(1,3);
    P_4(t,j,:) = Pg_x(1,4);
    P_5(t,j,:) = Pg_x(1,5);
    P_6(t,j,:) = Pg_x(1,6);
    P_7(t,j,:) = Pg_x(1,7);
end

z = 0;
while(fit(Pg)>1e-3)
    for z = 0:1:10
        if z == 10 || fit(Pg)<1e-3
            break
        else
            Z = MaxDT+z*10;  
            for t=1:Z                            % MaxDTΪ����������
                for i=1:N
                    v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(Pg_x-x(i,:));
                    % ȷ���ؽڲ�����λ��Χ(��������Ⱥ�������ռ�)
                    if t>1                       % ��Ҫ�����һ��λ�þͳ����߽�ʱ�����
                        x(i,:)=x(i,:)+v(i,:);    % �ж�ÿ�ε������λ���Ƿ�����
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
                               d2*(sin(x(i,4))*(sin(x(i,1))*sin(x(i,3)) -...
                               cos(x(i,1))*cos(x(i,2))*cos(x(i,3))) -...
                               cos(x(i,1))*cos(x(i,4))*sin(x(i,2)));
                    p_e(i,2) = d2*(sin(x(i,4))*(cos(x(i,1))*sin(x(i,3)) +...
                               cos(x(i,2))*cos(x(i,3))*sin(x(i,1))) +...
                               cos(x(i,4))*sin(x(i,1))*sin(x(i,2))) +...
                               d1*sin(x(i,1))*sin(x(i,2));
                    p_e(i,3) = d2*(cos(x(i,2))*cos(x(i,4)) -...
                               cos(x(i,3))*sin(x(i,2))*sin(x(i,4))) + d1*cos(x(i,2));
                    
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
                    p_e(Pg,1) = d1*cos(x(Pg,1))*sin(x(Pg,2)) -...
                                d2*(sin(x(Pg,4))*(sin(x(Pg,1))*sin(x(Pg,3)) -...
                                cos(x(Pg,1))*cos(x(Pg,2))*cos(x(Pg,3))) -...
                                cos(x(Pg,1))*cos(x(Pg,4))*sin(x(Pg,2)));
                    p_e(Pg,2) = d2*(sin(x(Pg,4))*(cos(x(Pg,1))*sin(x(Pg,3)) +...
                                cos(x(Pg,2))*cos(x(Pg,3))*sin(x(Pg,1))) +...
                                cos(x(Pg,4))*sin(x(Pg,1))*sin(x(Pg,2))) +...
                                d1*sin(x(Pg,1))*sin(x(Pg,2));
                    p_e(Pg,3) = d2*(cos(x(Pg,2))*cos(x(Pg,4)) -...
                                cos(x(Pg,3))*sin(x(Pg,2))*sin(x(Pg,4))) + d1*cos(x(Pg,2));
                    
                    fit(Pg) = sqrt((p_f(1)-p_e(Pg,1))^2+(p_f(2)-p_e(Pg,2))^2+...
                              (p_f(3)-p_e(Pg,3))^2)+alpha*((x(Pg,1)-q0(1))^2+...
                              (x(Pg,2)-q0(2))^2+(x(Pg,3)-q0(3))^2+(x(Pg,4)-q0(4))^2+...
                              (x(Pg,5)-q0(5))^2+(x(Pg,6)-q0(6))^2+(x(Pg,7)-q0(7))^2);
                    
                    if p(i)<fit(Pg)
                        Pg=i;
                        Pg_x=y(i,:);                 % ����Ⱥ�����λ��
                    end
                end
                record(:,:,t)=x(:,:);                % ��¼ÿһ��ѧϰ�����е��м�λ��ֵ
                %%%%%%%%%����ѧϰ����1��%%%%%%%%
                
                Pbest(t)=fit(Pg);                    % ����ÿһ����Ⱥ�����λ��
                
                P_1(t,j,:) = Pg_x(1,1);
                P_2(t,j,:) = Pg_x(1,2);
                P_3(t,j,:) = Pg_x(1,3);
                P_4(t,j,:) = Pg_x(1,4);
                P_5(t,j,:) = Pg_x(1,5);
                P_6(t,j,:) = Pg_x(1,6);
                P_7(t,j,:) = Pg_x(1,7);
                
                if z == 10 || fit(Pg)<1e-3
                    break
                end
            end
        end
    end
    if z == 10
        MAXDt = MaxDT + 10*10;
        break
    else
        MAXDt = MaxDT + z*10;
    end
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

if z == 0
    fprintf('����������%d\n',MaxDT);
else
    fprintf('��������: %d\n',MAXDt)
end

p_e(Pg,:);
a(j,:)=[Pg_x(1) Pg_x(2) Pg_x(3) Pg_x(4) Pg_x(5) Pg_x(6) Pg_x(7)];
Q{j}=Orochi_fkine(Pg_x(1),Pg_x(2),Pg_x(3),Pg_x(4),Pg_x(5),Pg_x(6),Pg_x(7));

figure(1)
plot(MaxDT,p_e(Pg,1),'ro-',MaxDT,p_e(Pg,2),'bo-',MaxDT,p_e(Pg,3),'go-')
hold on
end
toc  % �ú�����ʾ��ʱ����

H1 = cell2mat(Q);                              % ��Ԫ������ת��Ϊ����
T1 = double(nym_Link.fkine(a));                % ��е������

q1 = rad2deg(a(:,1));                          % 7���ؽڵĽǶ�ֵ������ת��Ϊ�Ƕ�
q2 = rad2deg(a(:,2));
q3 = rad2deg(a(:,3));
q4 = rad2deg(a(:,4));
q5 = rad2deg(a(:,5));
q6 = rad2deg(a(:,6));
q7 = rad2deg(a(:,7));

% �жϸõ���˶�ѧ�����ԣ������ſ˱�����ʽ�����Լ��ɲ�����
for n = 1:73
    J = Orochi_jacob(a(n,1),a(n,2),a(n,3),a(n,4),a(n,5),a(n,6),a(n,7));
    Rd(n,:) = rank(J);         % �ſ˱�����ʽ����
    D(n,:) = sqrt(det(J*J'));  % �ɲ�����
    if D > 1e-3
        fprintf('�����Ϊ�ɲ�����,�����ſ˱�����ʽ����=6\n')
    else
        Px=0.2*sin(pi*(n-1)/36); 
        Py=0.2*cos(pi*(n-1)/36); 
        %Pz=0.5; 
        Pz=0.5+(n-1)/720;
        D(n,:)=0;
        Point = Orochi_fkine(a(n,1),a(n,2),a(n,3),a(n,4),a(n,5),a(n,6),a(n,7));
        fprintf('��%d���Ϊ���ɲ�����,�����ſ˱�����ʽ����<6\n',n)
        fprintf('�õ�Ϊ�˶�ѧ����㣺%s %s %s,�õ���Ҫ������\n',Point(1,4),Point(2,4),Point(3,4))
        
        %******************* �����õ�����λ�� *******************%
        %%%%%%%%%%%%%%%%% ���ؿ��巨��������ɢ���� %%%%%%%%%%%%%%%
        delta=pi/180;
        A = unifrnd(a(n,1)-delta,a(n,1)+delta,[1,500]);    % ����q1��������
        B = unifrnd(a(n,2)-delta,a(n,2)+delta,[1,500]);    % ����q2��������
        C = unifrnd(a(n,3)-delta,a(n,3)+delta,[1,500]);    % ����q3��������
        D = unifrnd(a(n,4)-delta,a(n,4)+delta,[1,500]);    % ����q4��������
        E = unifrnd(a(n,5)-delta,a(n,5)+delta,[1,500]);    % ����q5��������
        F = unifrnd(a(n,6)-delta,a(n,6)+delta,[1,500]);    % ����q6��������
        G = unifrnd(a(n,7)-delta,a(n,7)+delta,[1,500]);    % ����q7��������
        Q = cell(500,7);
        
        for y=1:500
            Q{y}=[A(y) B(y) C(y) D(y) E(y) F(y) G(y)];
        end
        H=cell2mat(Q);
        
        for y=1:500
            T07 = Orochi_fkine(H(y,1),H(y,2),H(y,3),H(y,4),H(y,5),H(y,6),H(y,7));
            px = T07(1,4);
            py = T07(2,4);
            pz = T07(3,4); 
            Rn = sqrt((abs(px-Px))^2+(abs(py-Py))^2+(abs(pz-Pz))^2);
            Dn(y,:) = sqrt(det(j*j'));
            if Rn < 1e-4 && Dn > 1e-3
                Point(y,:) = [px py pz];
                j = Orochi_jacob(H(y,1),H(y,2),H(y,3),H(y,4),H(y,5),H(y,6),H(y,7));  
            else
                continue
            end
        end
        [I,K] = find(Dn==max(max(Dn)));
        fprintf('�������λ������Ϊ��%s %s %s ,�ɲ�����Ϊ��%s \n',Point(I,1),Point(I,2),Point(I,3),Dn(I,K))
        T1(1,4,n) = Point(I,1);
        T1(2,4,n) = Point(I,2);
        T1(3,4,n) = Point(I,3);
        q1(n,1) = H(I,1);
        q2(n,1) = H(I,2);
        q3(n,1) = H(I,3);
        q4(n,1) = H(I,4);
        q5(n,1) = H(I,5);
        q6(n,1) = H(I,6);
        q7(n,1) = H(I,7);
        D(n,1) = Dn(I,1);
    end
end


%%%%%%%%%%%%% ��ͼ %%%%%%%%%%%%%

figure(2)   %% ʵ�ʹ켣�������켣
s = 1:73;
scatter3(squeeze(T1(1,4,:)),squeeze(T1(2,4,:)),squeeze(T1(3,4,:)))  % �����ͼ
hold on

Px = 0.2*sin(pi*(s-1)/36);   % �����켣
Py = 0.2*cos(pi*(s-1)/36); 
% Pz=0.5+0*s;  
Pz = 0.5+(s-1)/720;  
plot3(Px,Py,Pz,'g',Px,Py,Pz,'ro')

 %%%%%%%%%%% ���ؽڽǶȲ�ֵ %%%%%%%%%%%
figure(3)  
S=1:73;
% �Ը����Ƕ�ֵ������������ֵ
subplot(4,2,1)
x1=linspace(1,73);
y1=spline(S,q1,x1);
plot(S,q1,'o-',x1,y1);
grid on

subplot(4,2,2)
x2=linspace(1,73);
y2=spline(S,q2,x2);
plot(S,q2,'o-',x2,y2);
grid on

subplot(4,2,3)
x3=linspace(1,73);
y3=spline(S,q3,x3);
plot(S,q3,'o-',x3,y3);
grid on

subplot(4,2,4)
x4=linspace(1,73);
y4=spline(S,q4,x4);
plot(S,q4,'o-',x4,y4);
grid on

subplot(4,2,5)
x5=linspace(1,73);
y5=spline(S,q5,x5);
plot(S,q5,'o-',x5,y5);
grid on

subplot(4,2,6)
x6=linspace(1,73);
y6=spline(S,q6,x6);
plot(S,q6,'o-',x6,y6);
grid on

subplot(4,2,7)
x7=linspace(1,73);
y7=spline(S,q7,x7);
plot(S,q7,'o-',x7,y7);
grid on

%%%%%%%%%%%% �ɲ�����ͼ�� %%%%%%%%%%%%
figure(4)
n=1:73;
D_ad = D*1/0.14; % �ɲ����ȹ�һ������
plot(n,D_ad,'ro-');
grid on

%%%%%%%%%%%% λ�õ���ͼ %%%%%%%%%%%%
t=1:length(P_1);
figure(5)
subplot(4,2,1)
plot(t,P_1(:,1),'o-')
grid on

subplot(4,2,2)
plot(t,P_2(:,1),'o-')
grid on

subplot(4,2,3)
plot(t,P_3(:,1),'o-')
grid on

subplot(4,2,4)
plot(t,P_4(:,1),'o-')
grid on

subplot(4,2,5)
plot(t,P_5(:,1),'o-')
grid on

subplot(4,2,6)
plot(t,P_6(:,1),'o-')
grid on

subplot(4,2,7)
plot(t,P_7(:,1),'o-')
grid on

figure(6)
X=1:length(Pbest);
Y=Pbest(1,:);
plot(X,Y,'o-')
% hold on 
% p1 = polyfit(X,Y,2);
% plot(X, polyval(p1, X));
% % axis([1900 2000 0 300]); 
% hold off

grid on
%%%%%%%%%%% 7���ؽڵĽǶȵ������� %%%%%%%%%%%
% t=1:length(P_1);
% for o=1:73
%     figure(o+6)
%     subplot(4,2,1)
%     plot(t,P_1(:,o),'o-')
%     grid on
%     
%     subplot(4,2,2)
%     plot(t,P_2(:,o),'o-')
%     grid on
%     
%     subplot(4,2,3)
%     plot(t,P_3(:,o),'o-')
%     grid on
%     
%     subplot(4,2,4)
%     plot(t,P_4(:,o),'o-')
%     grid on
%     
%     subplot(4,2,5)
%     plot(t,P_5(:,o),'o-')
%     grid on
%     
%     subplot(4,2,6)
%     plot(t,P_6(:,o),'o-')
%     grid on
%     
%     subplot(4,2,7)
%     plot(t,P_7(:,o),'o-')
%     grid on
% end

   
