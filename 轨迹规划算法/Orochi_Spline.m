%%%%%%%%%%%%%%%%%%%%%%% 三次样条曲线插补(笛卡尔空间) %%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = Orochi_Spline(t,tl,yl,dy0,dyn)
%%%%%%%%% 三次样条曲线 %%%%%%%%%
% "yl"为给定的路径点，"tl"为各路径点对应的时刻
% "dy0"为初始位置速度，"dyn"为终止位置速度
% 定义向量u，h和d
n=length(tl)-1;
mu=zeros(1,n);
lambda=zeros(1,n);
h=zeros(1,n);
d=zeros(1,n+1);
% 求时间间隔h
for i=1:n
    h(i)=tl(i+1)-tl(i);
end

% 求u
for i=1:n-1
    mu(i)=h(i)/(h(i)+h(i+1));
end
mu(n)=1;

% 求lambda
for i=2:n
    lambda(i)=h(i)/(h(i-1)+h(i));
end
lambda(1)=1;

% 求d
for i=2:n
    d(i)=6/(h(i-1)+h(i))*((yl(i+1)-yl(i))/h(i)-(yl(i)-yl(i-1))/h(i-1));
end
d(1)=6/h(1)*((yl(2)-yl(1))/h(1)-dy0);
d(n+1)=6/h(n)*(dyn-(yl(n+1)-yl(n))/h(n));

A=diag(lambda);
B=diag(mu);
C=2*eye(n+1);
D=C+[zeros(n,1) A;zeros(1,n+1)]+[zeros(1,n+1);B zeros(n,1)];
M=inv(D)*d';

% 计算输入时间t所对应的路径段m
for i=1:n+1
    ti=sum(tl(1:i));
    if tl(i)>t
        break;
    end
end
m=i;        % 得到路径段m
y=(tl(m)-t)^3/(6*h(m-1))*M(m-1)+(t-tl(m-1))^3/(6*h(m-1))*M(m)+...
    (yl(m-1)-(M(m-1)*h(m-1)^2)/6)*...
    (tl(m)-t)/h(m-1)+(yl(m)-(M(m)*h(m-1)^2)/6)*(t-tl(m-1))/h(m-1);
end

