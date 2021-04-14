%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人逆运动学求解(1) %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/10/21
% DH: MDH
% R(3x3)是姿态矩阵 P(3X1)是位置矩阵
% Example P = [0.1,0.2,0.3];
%         R = eye(3) 
%         q = nymikine(R,P)
% 锁定肘关节，第7个关节（冗余关节）用于提高机器人灵活性（解析解方法）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = Orochi_ik_analytic(R,P)
 p = num2cell(P);                     % P是位置矩阵
 [x_c,y_c,z_c] = deal(p{:});          % 将P矩阵的三个坐标分别赋给x_c,y_c,z_c
 [q1,q2,q3] = arm_angle(x_c,y_c,z_c); % 位置分离法求解出 q1 q2 q3的角度解析解
 
 % 前三个连杆（link 1 to 3）的DH参数，对应的是joint1 joint2 joint 4，3关节视为固定
 DH1 = [q1  0.34  0    90;
        q2  0     0.4  0;
        q3  0     0    -90
        ];    
 T_arm = FKinematics(DH1);
 R_arm = t2r(T_arm);   % rotation matrix of arm
 R6_3 = (R_arm.')*R;   % rotation of twist link
 
 % 计算 q4 q5 q6，角度分离法
 q4 = atan2(R6_3(2,3),R6_3(1,3))*180/pi;
 q5 = atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3))*180/pi;
 q6 = atan2(R6_3(3,2),R6_3(3,1))*180/pi;

 q = [q1 q2 q3 q4 q5 q6];
 
 % 计算 q1 q2 q3，位置分离法
    function [q1,q2,q3] = arm_angle(x_c,y_c,z_c)
        d0 = 0;
        d1 = 0.34;
        d3 = 0.4;
        s = z_c-d0;
        r = sqrt(x_c^2+y_c^2 );
        D = (s^2+r^2-d1^2-d3^2)/(2*d1*d3);
        
        % q1 q2 q3角度的解析解表达式
        q1 = atan2(y_c,x_c)*180/pi;
        q3 = atan2(sqrt(1-D^2),D)*180/pi;  % q3 = atan2(+-sqrt(1-D^2),D)
        q2 = atan2(s,r)-atan2(d3*sin(q3),d1+d3*sin(q3))*180/pi;
        
    end

    function Tr = FKinematics(DH)
        first = 1;
        last = size(DH,1);
        for i = first:last
            t = DH(i,1);
            d = DH(i,2);
            a = DH(i,3);
            al = DH(i,4);
            T(:,:,i) = [cos(t)     -sin(t)*cosd(al)    sin(t)*sind(al)    a*cos(t);
                        sin(t)     cos(t)*cosd(al)     -cos(t)*sind(al)   a*sin(t);
                        0          sind(al)            cosd(al)           d;
                        0          0                   0                  1];
        end
        Tr = eye(4);
        for i=first:last
            Tr = Tr*T(:,:,i);
        end
    end 
end


