%%%%%%%%%%%%%%%%%%%%%%%%% 抛物线轨迹拟合(关节空间) %%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/12/22
% DH: MDH 
% 7 DOF Cooperative Robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [y,dy,ddy,tp,tl] = Orochi_Para_curve(t,y0,a,dt)
% "tp"为抛物线拟合区的时间间隔，"tl"为直线段的时间间隔
% "dy"为直线段速度，"ddy"为抛物线拟合区加速度
% "y0"为路径点，"dt"为两相邻路径点之间的时间间隔
% "a"为拟合区加速度绝对值，"y"为"t"对应的输出值
n=length(dt);
if n==1
    ddy(1)=sign(y0(2)-y0(1))*a(1);
    ddy(2)=sign(y0(1)-y0(2))*a(2);
    tp(1)=dt/2-sqrt(ddy(1)^2*dt^2-4*ddy(1)*(y0(2)-y0(1)))/(2*ddy(1));
    tp(2)=tp(1);
    tl=dt-tp(1)-tp(2);
    dy=ddy(1)*tp(1);
    if t<=tp(1)
        y=y0(1)+ddy(1)/2*t^2; 
    else if t<=dt-tp(2)
            y=dy*(t-tp(1))+y0(1)+ddy(1)/2*tp(1)^2;
        else
            y=y0(2)+ddy(2)/2*(t-dt)^2;
        end
    end
else
    for i=1:n
        if i==1
            ddy(i)=sign(y0(i+1)-y0(i))*a(i);
            tp(i)=dt(i)-sqrt(dt(i)^2-2*(y0(i+1)-y0(i))/ddy(i));
            dy(i)=(y0(i+1)-y0(i))/(dt(i)-0.5*tp(i));
        else if i<n
                ddy(i)=sign(y0(i+1)-y0(i))*a(i);
                dy(i)=(y0(i+1)-y0(i))/dt(i);
                tp(i)=(dy(i)-dy(i-1))/ddy(i);
            else
                ddy(i)=sign(y0(i+1)-y0(i))*a(i);
                ddy(i+1)=sign(y0(i)-y0(i+1))*a(i+1);
                tp(i+1)=dt(i)-sqrt(dt(i)^2+2*(y0(i+1)-y0(i))/ddy(i+1));
                dy(i)=(y0(i+1)-y0(i))/(dt(i)-0.5*tp(i+1));
                tp(i)=(dy(i)-dy(i-1))/ddy(i);
            end
        end
    end
    for i=1:n
        if i==1
            tl(i)=dt(i)-tp(i)-0.5*tp(i+1);
        else if i<n
                tl(i)=dt(i)-0.5*tp(i)-0.5*tp(i+1);
            else
                tl(i)=dt(i)-tp(i+1)-0.5*tp(i);
            end
        end
    end
    for i=1:n
        A=sum(dt(1:i))+0.5*tp(i+1);
        if t<A
            break
        end
    end
    m=i-1;
    if m==0
        if t<=tp(1)
            y1=y0(1);
            y2=dy(1)*(tp(1)-0.5*tp(1))+y0(1);
            
            t1=0;
            t2=tp(1);
            a=(t2+t1-2*(y2-y1)/(ddy(1)*(t2-t1)))/2;
            b=y1-ddy(1)/2*(t1-a)^2;
            y=ddy(1)/2*(t-a)^2+b;
        else if t<=tp(1)+tl(1)
                y=dy(1)*(t-0.5*tp(1))+y0(1);
            else
                y1=dy(1)*(tp(1)+tl(1)-0.5*tp(1))+y0(1);
                y2=dy(2)*(dt(1)+0.5*tp(2)-dt(1))+y0(2);
                t1=tp(1)+tl(1);
                t2=dt(1)+0.5*tp(2);
                a=(t2+t1-2*(y2-y1)/(ddy(2)*(t2-t1)))/2;
                b=y1-ddy(2)/2*(t1-a)^2;
                y=ddy(2)/2*(t-a)^2+b;
            end
        end
    else
        B=sum(dt(1:m))+0.5*tp(m+1);
        if m<n-1
            if t<=B+tl(m+1)
                y=dy(m+1)*(t-(B-0.5*tp(m+1)))+y0(m+1);
            else
                y1=dy(m+1)*(B+tl(m+1)-(B-0.5*tp(m+1)))+y0(m+1);
                y2=dy(m+2)*0.5*tp(m+2)+y0(m+2);
                t1=B+tl(m+1);
                t2=B+tl(m+1)+tp(m+2);
                a=(t2+t1-2*(y2-y1)/(ddy(m+2)*(t2-t1)))/2;
                b=y1-ddy(m+2)/2*(t1-a)^2;
                y=ddy(m+2)/2*(t-a)^2+b;
            end
        else
            if t<=B+tl(m+1)
                y=dy(m+1)*(t-(B-0.5*tp(m+1)))+y0(m+1);
            else
                y1=dy(m+1)*(B+tl(m+1)-(B-0.5*tp(m+1)))+y0(m+1);
                y2=y0(m+2);
                t1=B+tl(m+1);
                t2=B+tl(m+1)+tp(m+2);
                a=(t2+t1-2*(y2-y1)/(ddy(m+2)*(t2-t1)))/2;
                b=y1-ddy(m+2)/2*(t1-a)^2;
                y=ddy(m+2)/2*(t-a)^2+b;
            end
        end
    end
end

