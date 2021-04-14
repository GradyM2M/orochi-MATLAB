%%%%%%%%%%%%%%%%%% ���м������ζ���ʽ��ֵ����(�ؽڿռ�) %%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/20
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=Orochi_Cubic_polynomial_without_mid_points(theta0,thetaf,tf)
% theta0=input('Enter the theta0:');   
% thetaf=input('Enter the thetaf:');
% tf=input('Enter the t:');
% theta0 = 0;
% thetaf = pi/2;
% tf = 4
% ������β�ֵ������4����ֵϵ��a0,a1,a2��a3
a0=theta0;
a1=0;
a2=3/tf^2*(thetaf-theta0);
a3=-2/tf^3*(thetaf-theta0);
t=0:0.01:tf;
n=length(t);
for i=1:n
    theta(i)=a0+a1*t(i)+a2*t(i)^2+a3*t(i)^3;
end
figure('Name','���м�����ζ���ʽ�岹����')
plot(t,theta)
grid on
xlabel('Time(s)');ylabel('��(rad)');
axis([0 tf theta0 thetaf])
end

% pp=interp1(x,y,'spline','pp') 
% breaks=pp.breaks 
% coefs=pp.coefs 
