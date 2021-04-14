%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7轴机械臂轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/17
% DH: SDH
% 轨迹规划，驱动kinovaGen3机器人运动
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
%% 读取机械臂模型，并设置仿真参数
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
numJoints = numel(currentRobotJConfig);
endEffector = "EndEffector_Link";
timeStep = 0.1;  % seconds
toolSpeed = 0.1; % m/s
jointInit = currentRobotJConfig;

%% 定义初始、终止任务点，设置碰撞距离
taskInit = getTransform(robot,jointInit,endEffector);          

taskFinal = trvec2tform([0.4,0,0.6])*axang2tform([0 1 0 pi]);  


distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal)); 
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');

tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

%% 运动学逆解
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% 轨迹规划
initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);

ctrlpoints = [jointInit',jointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
jointWaypoints = bsplinepolytraj(jointConfigArray,timeInterval,1);

jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');

q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));
[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);

show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);


%% 仿真动画
for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20)
    drawnow;
end

% Return to initial configuration
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
 
for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tJoint,stateJoint(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',20)
    drawnow;
end

figure(2);
grid on;
plot(tTask,stateTask(:,1:numJoints));
hold all;
plot(tTask(1:numJoints),stateTask(1:numJoints),"--");
title("Joint Position vs Reference ");
xlabel("Time (s)")
ylabel("Position (rad)");
