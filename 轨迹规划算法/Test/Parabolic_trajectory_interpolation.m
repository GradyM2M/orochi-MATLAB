%%%%%%%%%%%%%%%%%%%%%%%%%%% 抛物线过渡域轨迹测试 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/22
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
y0=[30 15 25 10 18];
dt=[2 2 2 2];
a=[20 20 20 20 20];
ts=sum(dt);
t=0:0.01:ts;
n=length(t);
for i=1:n
    [y(i),dy,~,tp,~] = Orochi_Para_curve(t(i),y0,a,dt);
end
figure('Name','带有抛物线过渡的线性插值曲线');
for i=1:length(y0)
    t0(i)=sum(dt(1:i-1));
end
t0(1)=0.5*tp(1);
t0(end)=t0(end)-0.5*tp(end);
plot([0 t0 ts],[y0(1) y0 y0(end)],'r-o','MarkerFaceColor','r')

hold on 
plot(t,y,'b')
grid on 
xlabel('Time(s)');ylabel('θ(deg)');
xlim([0 ts])