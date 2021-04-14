%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7轴机械臂轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/19
% DH: SDH
% 轨迹规划，驱动kinovaGen3机器人运动
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 初始化
clear all
clc
close all

%% 路径点
createWaypointData;
% donate_one_second
%% 运动学逆解
ik = inverseKinematics('RigidBodyTree',gen3);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = gen3.homeConfiguration;

%% 绘制轨迹点
plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(gen3,jointAnglesHome','Frames','off','PreservePlot',false);
xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
hold on
if plotMode == 1
    hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
end
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);


%% 生成轨迹
 
[q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
             'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
             'EndTime',repmat(diff(waypointTimes),[3 1]));

% [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ...
%             'VelocityBoundaryCondition',waypointVels);
% [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
%             'VelocityBoundaryCondition',waypointVels, ...
%             'AccelerationBoundaryCondition',waypointAccels);
% 


% Show the full trajectory with the rigid body tree
set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));

%% 位置速度曲线
 plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% 运动仿真
for idx = 1:numel(trajTimes) 
    % IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;

    % Show the robot
    show(gen3,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    drawnow    
end