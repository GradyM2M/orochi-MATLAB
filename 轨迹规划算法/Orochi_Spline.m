%%%%%%%%%%%%%%%%%%%%%%% �����������߲岹(�ѿ����ռ�) %%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/25
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = Orochi_Spline(t,tl,yl,dy0,dyn)
%%%%%%%%% ������������ %%%%%%%%%
% "yl"Ϊ������·���㣬"tl"Ϊ��·�����Ӧ��ʱ��
% "dy0"Ϊ��ʼλ���ٶȣ�"dyn"Ϊ��ֹλ���ٶ�
% ��������u��h��d
n=length(tl)-1;
mu=zeros(1,n);
lambda=zeros(1,n);
h=zeros(1,n);
d=zeros(1,n+1);
% ��ʱ����h
for i=1:n
    h(i)=tl(i+1)-tl(i);
end

% ��u
for i=1:n-1
    mu(i)=h(i)/(h(i)+h(i+1));
end
mu(n)=1;

% ��lambda
for i=2:n
    lambda(i)=h(i)/(h(i-1)+h(i));
end
lambda(1)=1;

% ��d
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

% ��������ʱ��t����Ӧ��·����m
for i=1:n+1
    ti=sum(tl(1:i));
    if tl(i)>t
        break;
    end
end
m=i;        % �õ�·����m
y=(tl(m)-t)^3/(6*h(m-1))*M(m-1)+(t-tl(m-1))^3/(6*h(m-1))*M(m)+...
    (yl(m-1)-(M(m-1)*h(m-1)^2)/6)*...
    (tl(m)-t)/h(m-1)+(yl(m)-(M(m)*h(m-1)^2)/6)*(t-tl(m-1))/h(m-1);
end

