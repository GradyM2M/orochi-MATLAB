%%%%%%%%%%%%%%%%%%%%%%%%%%% 三次样条曲线插补测试 %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
tl = [0 2 4 6 8];
Pl = [4 7 8;7 10 2;10 15 10;13 20 -2;15 15 22]';
dP0 = [0 0 0];
dPn = [0 0 0];
t = 0:0.01:tl(end);
n = length(t);
for i=1:n
    x(i) = Orochi_Spline(t(i),tl,Pl(1,:),dP0(1),dPn(1));
    y(i) = Orochi_Spline(t(i),tl,Pl(2,:),dP0(2),dPn(2));
    z(i) = Orochi_Spline(t(i),tl,Pl(3,:),dP0(3),dPn(3));
end
P = [x;y;z];

figure('Name','三次样条轨迹');
plot3(Pl(1,:),Pl(2,:),Pl(3,:),'ro','MarkerFaceColor','r')

hold on
plot3(P(1,:),P(2,:),P(3,:))
grid;
xlabel('X');ylabel('Y');zlabel('Z');
figure('Name','位置坐标随时间变化曲线');
subplot(3,1,1)
plot(t,x)
grid;
xlabel('时间(s)');ylabel('Px');
subplot(3,1,2)
plot(t,y)
grid;
xlabel('时间(s)');ylabel('Py');
subplot(3,1,3)
plot(t,z)
grid;
xlabel('时间(s)');ylabel('Pz');