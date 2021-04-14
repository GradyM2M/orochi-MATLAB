%%%%%%%%%%%%%%%%%%%%%%%%%% 7���е�۹켣�滮����� %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/13
% DH: SDH
% �ڿ�����ײ�Ļ����Ͻ��й켣�滮
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
clc
close all
%% ��������ƽ̨
platform1 = collisionBox(0.5,0.5,0.25);
platform1.Pose = trvec2tform([-0.5 0.4 0.2]);

platform2 = collisionBox(0.5,0.5,0.25);
platform2.Pose = trvec2tform([0.5 0.2 0.2]);

%% ���һ���ƾߣ���ģΪһ������
lightFixture = collisionSphere(0.1);
lightFixture.Pose = trvec2tform([.2 0 1]);

%% �洢�ڵ�Ԫ�������У��Ա������ײ���
worldCollisionArray = {platform1 platform2 lightFixture};

exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
show(robot,homeConfiguration(robot),"Parent",ax);
% lbr = importrobot('iiwa14.urdf'); % 14 kg payload version
% show(lbr)


%% ����һ����ͻ��������
collisionArray = exampleHelperManipCollisionsFromVisuals(robot);

startPose = trvec2tform([-0.5,0.5,0.4])*axang2tform([1 0 0 pi]);
endPose = trvec2tform([0.5,0.2,0.4])*axang2tform([1 0 0 pi]);

%% ʹ�ù̶������������ȷ�����ظ��Ľ��
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
startConfig = ik("EndEffector_Link",startPose,weights,robot.homeConfiguration);
endConfig = ik("EndEffector_Link",endPose,weights,robot.homeConfiguration);



%% ��ʾ��ʼ������λ��
show(robot,startConfig);
show(robot,endConfig);


q = trapveltraj([homeConfiguration(robot),startConfig,endConfig],200,"EndTime",2);

%% ��ʼ�����
isCollision = false(length(q),1);           % ���ÿ�������Ƿ��г�ͻ
selfCollisionPairIdx = cell(length(q),1);   % �ṩ��ײ�Ļ���
worldCollisionPairIdx = cell(length(q),1);  % �ṩ��ײ�Ļ���

for i = 1:length(q)
    [isCollision(i),selfCollisionPairIdx{i},worldCollisionPairIdx{i}] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q(:,i),false);
end
isTrajectoryInCollision = any(isCollision);

problemIdx1 = find(isCollision,1);
problemIdx2 = find(isCollision,1,"last");

%% ʶ�����
problemBodies1 = [selfCollisionPairIdx{problemIdx1} worldCollisionPairIdx{problemIdx1}*[1 0]'];
problemBodies2 = [selfCollisionPairIdx{problemIdx2} worldCollisionPairIdx{problemIdx2}*[1 0]'];

%% ���ӻ�����
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

%% ��ӻ�����
show(robot,q(:,problemIdx1),"Parent",ax,"PreservePlot",false);
exampleHelperHighlightCollisionBodies(robot,problemBodies1,ax);
show(robot,q(:,problemIdx2),"Parent"',ax);
exampleHelperHighlightCollisionBodies(robot,problemBodies2,ax);

intermediatePose1 = trvec2tform([-.3 -.2 .6])*axang2tform([0 1 0 -pi/4]); % Out and around the sphere
intermediatePose2 = trvec2tform([0.2,0.2,0.6])*axang2tform([1 0 0 pi]); % Come in from above

intermediateConfig1 = ik("EndEffector_Link",intermediatePose1,weights,q(:,problemIdx1));
intermediateConfig2 = ik("EndEffector_Link",intermediatePose2,weights,q(:,problemIdx2));

%% չʾ�µ��м�����
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
show(robot,intermediateConfig1,"Parent",ax,"PreservePlot",false);
show(robot,intermediateConfig2,"Parent",ax);

[q,qd,qdd,t] = trapveltraj([homeConfiguration(robot),intermediateConfig1,startConfig,intermediateConfig2,endConfig],200,"EndTime",2);


%% ��ʼ�����
isCollision = false(length(q),1); % Check whether each pose is in collision
collisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision
for i = 1:length(q)
    [isCollision(i),collisionPairIdx{i}] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q(:,i),false);
end
isTrajectoryInCollision = any(isCollision);

%% ���ƻ���
ax2 = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

%% �������������ʼ��������ʾ����
show(robot,startConfig,"Parent",ax2);

%% ������Ĵ�С
axis equal

%% ѭ��ͨ������λ��
for i = 1:length(q)
    show(robot,q(:,i),"Parent",ax2,"PreservePlot",false);
    
    % Update the figure    
    drawnow
end

figure
plot(t,q)
xlabel("Time")
ylabel("Joint Position")