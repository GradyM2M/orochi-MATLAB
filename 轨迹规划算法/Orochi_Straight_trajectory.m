%%%%%%%%%%%%%%%%%%%%%%%%%% 笛卡尔空间直线轨迹插补 %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/01/09
% DH: SDH
% 7 DOF Cooperative Robot
% 给定已知起始点和终止点，通过逆运动学计算和插值进行轨迹规划
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=Orochi_Straight_trajectory(Px,Py,Pz,px,py,pz)
% Px = 0.1; Py = -0.1;  Pz = 0.3;
% px = 0.5; py = 0.3;   pz = 0.5;

P = [Px Py Pz];
p = [px py pz];
L1 = Link([0     0     0     -pi/2  0]);
L2 = Link([0     0     0     pi/2   0]);
L3 = Link([0     0.5   0     -pi/2  0]);
L4 = Link([0     0     0     pi/2   0]);
L5 = Link([0     0.4   0     -pi/2  0]);
L6 = Link([0     0     0     pi/2   0]);
L7 = Link([0     0.1   0     0      0]);
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','Orochi'); %连接连杆，机器人取名manman
T1=transl(P(1,1),P(1,2),P(1,3));%根据给定起始点，得到起始点位姿
T2=transl(p(1,1),p(1,2),p(1,3));%根据给定终止点，得到终止点位姿

T=ctraj(T1,T2,50);
Tj=transl(T);
figure(1)
plot3(Tj(:,1),Tj(:,2),Tj(:,3)); % 输出末端轨迹
grid on;
q=robot.ikine(T);
hold on
robot.plot(q);%动画演示

q1=q(:,1);
q2=q(:,2);
q3=q(:,3);
q4=q(:,4);
q5=q(:,5);
q6=q(:,6);
q7=q(:,7);

figure(2)
plot3(Tj(:,1),Tj(:,2),Tj(:,3),'b-o');%输出末端轨迹
grid on

figure(3)
subplot(7,1,1)
for i=1:1:50
if q1(i)>=0
    q1(i)=-q1(i);
else
    q1(i)=q1(i);
end
if q2(i)>=0
    q2(i)=-q2(i);
    else
    q2(i)=q2(i);
end
if q3(i)>=0
    q3(i)=-q3(i);
    else
    q3(i)=q3(i);
end
if q4(i)>=0
    q4(i)=-q4(i);
    else
    q4(i)=q4(i);
end
if q5(i)<=0
    q5(i)=-q5(i);
    else
    q5(i)=q5(i);
end
if q6(i)<=0
    q6(i)=-q6(i);
    else
    q6(i)=q6(i);
end
if q7(i)<=0
    q7(i)=-q7(i);
    else
    q7(i)=q7(i);
end
end
plot(q1*180/pi);
grid on
title('关节1');

subplot(7,1,2)
plot(q2*180/pi);
grid on
title('关节2');

subplot(7,1,3)
plot(q3*180/pi);
grid on
title('关节3');

subplot(7,1,4)
plot(q4*180/pi);
grid on
title('关节4');

subplot(7,1,5)
plot(q5*180/pi);
grid on
title('关节5');

subplot(7,1,6)
plot(q6*180/pi);
grid on
title('关节6');

subplot(7,1,7)
plot(q7*180/pi);
grid on
title('关节7');
end

