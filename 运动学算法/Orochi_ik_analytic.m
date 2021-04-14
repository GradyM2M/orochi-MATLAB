%%%%%%%%%%%%%%%%%%%%%%% 7DOFЭ�����������˶�ѧ���(1) %%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2019/10/21
% DH: MDH
% R(3x3)����̬���� P(3X1)��λ�þ���
% Example P = [0.1,0.2,0.3];
%         R = eye(3) 
%         q = nymikine(R,P)
% ������ؽڣ���7���ؽڣ�����ؽڣ�������߻���������ԣ������ⷽ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = Orochi_ik_analytic(R,P)
 p = num2cell(P);                     % P��λ�þ���
 [x_c,y_c,z_c] = deal(p{:});          % ��P�������������ֱ𸳸�x_c,y_c,z_c
 [q1,q2,q3] = arm_angle(x_c,y_c,z_c); % λ�÷��뷨���� q1 q2 q3�ĽǶȽ�����
 
 % ǰ�������ˣ�link 1 to 3����DH��������Ӧ����joint1 joint2 joint 4��3�ؽ���Ϊ�̶�
 DH1 = [q1  0.34  0    90;
        q2  0     0.4  0;
        q3  0     0    -90
        ];    
 T_arm = FKinematics(DH1);
 R_arm = t2r(T_arm);   % rotation matrix of arm
 R6_3 = (R_arm.')*R;   % rotation of twist link
 
 % ���� q4 q5 q6���Ƕȷ��뷨
 q4 = atan2(R6_3(2,3),R6_3(1,3))*180/pi;
 q5 = atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3))*180/pi;
 q6 = atan2(R6_3(3,2),R6_3(3,1))*180/pi;

 q = [q1 q2 q3 q4 q5 q6];
 
 % ���� q1 q2 q3��λ�÷��뷨
    function [q1,q2,q3] = arm_angle(x_c,y_c,z_c)
        d0 = 0;
        d1 = 0.34;
        d3 = 0.4;
        s = z_c-d0;
        r = sqrt(x_c^2+y_c^2 );
        D = (s^2+r^2-d1^2-d3^2)/(2*d1*d3);
        
        % q1 q2 q3�ǶȵĽ�������ʽ
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


