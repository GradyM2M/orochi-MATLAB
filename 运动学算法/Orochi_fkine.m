%%%%%%%%%%%%%%%%%%%%%%% 7DOF协作机器人正运动学求解 %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/10/13
% DH: MDH or SDH
% MDH = 1; SDH = 0; % md取1时，DH参数为修正参数；md取0时，DH参数为标准参数
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T07] = Orochi_fkine(q1,q2,q3,q4,q5,q6,q7)
md = 0;
if md == 1
    %      theta  d      a    alpha
    MDH = [q1     0      0    0;
           q2     0      0    pi/2;
           q3     0.4    0    -pi/2;
           q4     0      0    -pi/2;
           q5     0.4    0    pi/2;
           q6     0      0    pi/2;
           q7     0      0    -pi/2];
    
    T01 = [cos(MDH(1,1))                 -sin(MDH(1,1))                0                MDH(1,3);
           sin(MDH(1,1))*cos(MDH(1,4))   cos(MDH(1,1))*cos(MDH(1,4))   -sin(MDH(1,4))   -sin(MDH(1,4))*MDH(1,2);
           sin(MDH(1,1))*sin(MDH(1,4))   cos(MDH(1,1))*sin(MDH(1,4))   cos(MDH(1,4))    cos(MDH(1,4))*MDH(1,2);
           0                             0                             0                1];
    T12 = [cos(MDH(2,1))                 -sin(MDH(2,1))                0                MDH(2,3);
           sin(MDH(2,1))*cos(MDH(2,4))   cos(MDH(2,1))*cos(MDH(2,4))   -sin(MDH(2,4))   -sin(MDH(2,4))*MDH(2,2);
           sin(MDH(2,1))*sin(MDH(2,4))   cos(MDH(2,1))*sin(MDH(2,4))   cos(MDH(2,4))    cos(MDH(2,4))*MDH(2,2);
           0                             0                             0                1];
    T23 = [cos(MDH(3,1))                 -sin(MDH(3,1))                0                MDH(3,3);
           sin(MDH(3,1))*cos(MDH(3,4))   cos(MDH(3,1))*cos(MDH(3,4))   -sin(MDH(3,4))   -sin(MDH(3,4))*MDH(3,2);
           sin(MDH(3,1))*sin(MDH(3,4))   cos(MDH(3,1))*sin(MDH(3,4))   cos(MDH(3,4))    cos(MDH(3,4))*MDH(3,2);
           0                             0                             0                1];
    T34 = [cos(MDH(4,1))                 -sin(MDH(4,1))                0                MDH(4,3);
           sin(MDH(4,1))*cos(MDH(4,4))   cos(MDH(4,1))*cos(MDH(4,4))   -sin(MDH(4,4))   -sin(MDH(4,4))*MDH(4,2);
           sin(MDH(4,1))*sin(MDH(4,4))   cos(MDH(4,1))*sin(MDH(4,4))   cos(MDH(4,4))    cos(MDH(4,4))*MDH(4,2);
           0                             0                             0                1];
    T45 = [cos(MDH(5,1))                 -sin(MDH(5,1))                0                MDH(1,3);
           sin(MDH(5,1))*cos(MDH(5,4))   cos(MDH(5,1))*cos(MDH(5,4))   -sin(MDH(5,4))   -sin(MDH(5,4))*MDH(5,2);
           sin(MDH(5,1))*sin(MDH(5,4))   cos(MDH(5,1))*sin(MDH(5,4))   cos(MDH(5,4))    cos(MDH(5,4))*MDH(5,2);
           0                             0                             0                1];
    T56 = [cos(MDH(6,1))                 -sin(MDH(6,1))                0                MDH(6,3);
           sin(MDH(6,1))*cos(MDH(6,4))   cos(MDH(6,1))*cos(MDH(6,4))   -sin(MDH(6,4))   -sin(MDH(6,4))*MDH(6,2);
           sin(MDH(6,1))*sin(MDH(6,4))   cos(MDH(6,1))*sin(MDH(6,4))   cos(MDH(6,4))    cos(MDH(6,4))*MDH(6,2);
           0                             0                             0                1];
    T67 = [cos(MDH(7,1))                 -sin(MDH(7,1))                0                MDH(7,3);
           sin(MDH(7,1))*cos(MDH(7,4))   cos(MDH(7,1))*cos(MDH(7,4))   -sin(MDH(7,4))   -sin(MDH(7,4))*MDH(7,2);
           sin(MDH(7,1))*sin(MDH(7,4))   cos(MDH(7,1))*sin(MDH(7,4))   cos(MDH(7,4))    cos(MDH(7,4))*MDH(7,2);
           0                             0                             0                1];
    T07 = T01*T12*T23*T34*T45*T56*T67;
elseif md == 0
    %      theta  d      a    alpha
    SDH = [q1     0      0    -pi/2;
           q2     0      0    pi/2;
           q3     0.4    0    -pi/2;
           q4     0      0    pi/2;
           q5     0.4    0    -pi/2;
           q6     0      0    pi/2;
           q7     0      0    0];
    
    T01 = [cos(SDH(1,1))  -sin(SDH(1,1))*cos(SDH(1,4))  sin(SDH(1,1))*sin(SDH(1,4))   SDH(1,3)*cos(SDH(1,1));
           sin(SDH(1,1))  cos(SDH(1,1))*cos(SDH(1,4))   -cos(SDH(1,1))*sin(SDH(1,4))  SDH(1,3)*sin(SDH(1,1));
           0              sin(SDH(1,4))                 cos(SDH(1,4))                 SDH(1,2);
           0              0                             0                             1];
    T12 = [cos(SDH(2,1))  -sin(SDH(2,1))*cos(SDH(2,4))  sin(SDH(2,1))*sin(SDH(2,4))   SDH(2,3)*cos(SDH(2,1));
           sin(SDH(2,1))  cos(SDH(2,1))*cos(SDH(2,4))   -cos(SDH(2,1))*sin(SDH(2,4))  SDH(2,3)*sin(SDH(2,1));
           0              sin(SDH(2,4))                 cos(SDH(2,4))                 SDH(2,2);
           0              0                             0                             1];
    T23 = [cos(SDH(3,1))  -sin(SDH(3,1))*cos(SDH(3,4))  sin(SDH(3,1))*sin(SDH(3,4))   SDH(3,3)*cos(SDH(3,1));
           sin(SDH(3,1))  cos(SDH(3,1))*cos(SDH(3,4))   -cos(SDH(3,1))*sin(SDH(3,4))  SDH(3,3)*sin(SDH(3,1));
           0              sin(SDH(3,4))                 cos(SDH(3,4))                 SDH(3,2);
           0              0                             0                             1];
    T34 = [cos(SDH(4,1))  -sin(SDH(4,1))*cos(SDH(4,4))  sin(SDH(4,1))*sin(SDH(4,4))   SDH(4,3)*cos(SDH(4,1));
           sin(SDH(4,1))  cos(SDH(4,1))*cos(SDH(4,4))   -cos(SDH(4,1))*sin(SDH(4,4))  SDH(4,3)*sin(SDH(4,1));
           0              sin(SDH(4,4))                 cos(SDH(4,4))                 SDH(4,2);
           0              0                             0                             1];
    T45 = [cos(SDH(5,1))  -sin(SDH(5,1))*cos(SDH(5,4))  sin(SDH(5,1))*sin(SDH(5,4))   SDH(5,3)*cos(SDH(5,1));
           sin(SDH(5,1))  cos(SDH(5,1))*cos(SDH(5,4))   -cos(SDH(5,1))*sin(SDH(5,4))  SDH(5,3)*sin(SDH(5,1));
           0              sin(SDH(5,4))                 cos(SDH(5,4))                 SDH(5,2);
           0              0                             0                             1];
    T56 = [cos(SDH(6,1))  -sin(SDH(6,1))*cos(SDH(6,4))  sin(SDH(6,1))*sin(SDH(6,4))   SDH(6,3)*cos(SDH(6,1));
           sin(SDH(6,1))  cos(SDH(6,1))*cos(SDH(6,4))   -cos(SDH(6,1))*sin(SDH(6,4))  SDH(6,3)*sin(SDH(6,1));
           0              sin(SDH(6,4))                 cos(SDH(6,4))                 SDH(6,2);
           0              0                             0                             1];
    T67 = [cos(SDH(7,1))  -sin(SDH(7,1))*cos(SDH(7,4))  sin(SDH(7,1))*sin(SDH(7,4))   SDH(7,3)*cos(SDH(7,1));
           sin(SDH(7,1))  cos(SDH(7,1))*cos(SDH(7,4))   -cos(SDH(7,1))*sin(SDH(7,4))  SDH(7,3)*sin(SDH(7,1));
           0              sin(SDH(7,4))                 cos(SDH(7,4))                 SDH(7,2);
           0              0                             0                             1];
    T07 = T01*T12*T23*T34*T45*T56*T67;
% Px = T07(1,4); Py = T07(2,4); Pz=T07(3,4);
end