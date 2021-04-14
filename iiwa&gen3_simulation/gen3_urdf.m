%%%%%%%%%%%%%%%%%%%%%%%%%% 7轴机械臂轨迹规划与避障 %%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/13
% DH: SDH
% 在考虑碰撞的基础上进行轨迹规划
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear 
clc
close all
%% 创建两个平台
platform1 = collisionBox(0.5,0.5,0.25);
platform1.Pose = trvec2tform([-0.5 0.4 0.2]);

platform2 = collisionBox(0.5,0.5,0.25);
platform2.Pose = trvec2tform([0.5 0.2 0.2]);

%% 添加一个灯具，建模为一个球体
lightFixture = collisionSphere(0.1);
lightFixture.Pose = trvec2tform([.2 0 1]);

%% 存储在单元格数组中，以便进行碰撞检查
worldCollisionArray = {platform1 platform2 lightFixture};

exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
show(robot,homeConfiguration(robot),"Parent",ax);
% lbr = importrobot('iiwa14.urdf'); % 14 kg payload version
% show(lbr)


%% 生成一个冲突对象数组
collisionArray = exampleHelperManipCollisionsFromVisuals(robot);

startPose = trvec2tform([-0.5,0.5,0.4])*axang2tform([1 0 0 pi]);
endPose = trvec2tform([0.5,0.2,0.4])*axang2tform([1 0 0 pi]);

%% 使用固定的随机数，以确保可重复的结果
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
startConfig = ik("EndEffector_Link",startPose,weights,robot.homeConfiguration);
endConfig = ik("EndEffector_Link",endPose,weights,robot.homeConfiguration);



%% 显示初始和最终位置
show(robot,startConfig);
show(robot,endConfig);


q = trapveltraj([homeConfiguration(robot),startConfig,endConfig],200,"EndTime",2);

%% 初始化输出
isCollision = false(length(q),1);           % 检查每个姿势是否有冲突
selfCollisionPairIdx = cell(length(q),1);   % 提供碰撞的机体
worldCollisionPairIdx = cell(length(q),1);  % 提供碰撞的机体

for i = 1:length(q)
    [isCollision(i),selfCollisionPairIdx{i},worldCollisionPairIdx{i}] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q(:,i),false);
end
isTrajectoryInCollision = any(isCollision);

problemIdx1 = find(isCollision,1);
problemIdx2 = find(isCollision,1,"last");

%% 识别刚体
problemBodies1 = [selfCollisionPairIdx{problemIdx1} worldCollisionPairIdx{problemIdx1}*[1 0]'];
problemBodies2 = [selfCollisionPairIdx{problemIdx2} worldCollisionPairIdx{problemIdx2}*[1 0]'];

%% 可视化环境
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

%% 添加机器人
show(robot,q(:,problemIdx1),"Parent",ax,"PreservePlot",false);
exampleHelperHighlightCollisionBodies(robot,problemBodies1,ax);
show(robot,q(:,problemIdx2),"Parent"',ax);
exampleHelperHighlightCollisionBodies(robot,problemBodies2,ax);

intermediatePose1 = trvec2tform([-.3 -.2 .6])*axang2tform([0 1 0 -pi/4]); % Out and around the sphere
intermediatePose2 = trvec2tform([0.2,0.2,0.6])*axang2tform([1 0 0 pi]); % Come in from above

intermediateConfig1 = ik("EndEffector_Link",intermediatePose1,weights,q(:,problemIdx1));
intermediateConfig2 = ik("EndEffector_Link",intermediatePose2,weights,q(:,problemIdx2));

%% 展示新的中间姿势
ax = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);
show(robot,intermediateConfig1,"Parent",ax,"PreservePlot",false);
show(robot,intermediateConfig2,"Parent",ax);

[q,qd,qdd,t] = trapveltraj([homeConfiguration(robot),intermediateConfig1,startConfig,intermediateConfig2,endConfig],200,"EndTime",2);


%% 初始化输出
isCollision = false(length(q),1); % Check whether each pose is in collision
collisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision
for i = 1:length(q)
    [isCollision(i),collisionPairIdx{i}] = exampleHelperManipCheckCollisions(robot,collisionArray,worldCollisionArray,q(:,i),false);
end
isTrajectoryInCollision = any(isCollision);

%% 绘制环境
ax2 = exampleHelperVisualizeCollisionEnvironment(worldCollisionArray);

%% 将机器人在其初始构型中显示出来
show(robot,startConfig,"Parent",ax2);

%% 更新轴的大小
axis equal

%% 循环通过其他位置
for i = 1:length(q)
    show(robot,q(:,i),"Parent",ax2,"PreservePlot",false);
    
    % Update the figure    
    drawnow
end

figure
plot(t,q)
xlabel("Time")
ylabel("Joint Position")