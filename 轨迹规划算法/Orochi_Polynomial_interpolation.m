%%%%%%%%%%%%%%%%%%%%%%% 五次多项式插值曲线(关节空间) %%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = Orochi_Polynomial_interpolation(s0,sf,ts)
% s0=input('Enter the s0:');
% sf=input('Enter the sf:');
% ts=input('Enter the t:');
% s0 = 0;
% sf = pi;
% ts = 4;
t = 0:0.01:ts;
[s,sd,sdd] = tpoly(s0,sf,t);

figure('Name','五次多项式插值曲线')
subplot(3,1,1);
plot(t,s)
grid on
xlabel('时间(s)');ylabel('位移(rad)');
xlim([0,ts]);
subplot(3,1,2);
plot(t,sd);
grid on
xlabel('时间(s)');ylabel('速度(rad/s)');
xlim([0,ts]);
subplot(3,1,3);
plot(t,sdd)
grid on
xlabel('时间(s)');ylabel('加速度(rad/s^2)');
xlim([0,ts]);
end



