%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7���е�۹켣�滮 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/17
% DH: SDH
% �켣�滮������kinovaGen3�������˶�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
%% ���ػ�����ģ��
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);%�����˹ؽ�����
 
numJoints = numel(currentRobotJConfig);%�����˹ؽ�����
endEffector = "EndEffector_Link";
 
timeStep = 0.1; % ����
toolSpeed = 0.1; % �ٶ�
 
jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);%��ʼ
taskFinal = trvec2tform([0.4,-0.4,0.6])*axang2tform([0 1 0 pi]);%ĩ��
 
%% Task-Space Trajectory
distance= norm(tform2trvec(taskInit)-tform2trvec(taskFinal));%����
 
initTime = 0;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];
% ���в�ֵ
[taskWaypoints,taskVelocities] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes); 
% Control Task-Space Motion
% PID������
tsMotionModel = taskSpaceMotionModel('RigidBodyTree',robot,'EndEffectorName','EndEffector_Link');
tsMotionModel.Kp(1:3,1:3) = 0;
tsMotionModel.Kd(1:3,1:3) = 0;
 
q0 = currentRobotJConfig;  % ��ʼ״̬
qd0 = zeros(size(q0));

%% ģ��������˶�
[tTask,stateTask] = ode15s(@(t,state) exampleHelperTimeBasedTaskInputs(tsMotionModel,timeInterval,taskInit,taskFinal,t,state),timeInterval,[q0; qd0]);
 
 
%% joint space
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

%% �˶�ѧ���
initialGuess = wrapToPi(jointInit);
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);
jointFinal = wrapToPi(jointFinal);

%% ��ֵ
ctrlpoints = [jointInit',jointFinal'];
jointConfigArray = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
jointWaypoints = bsplinepolytraj(jointConfigArray,timeInterval,1);
 
jsMotionModel = jointSpaceMotionModel('RigidBodyTree',robot,'MotionType','PDControl');
q0 = currentRobotJConfig; 
qd0 = zeros(size(q0));

%% ģ��������˶�
[tJoint,stateJoint] = ode15s(@(t,state) exampleHelperTimeBasedJointInputs(jsMotionModel,timeInterval,jointConfigArray,t,state),timeInterval,[q0; qd0]);
 
 
% ��ʼ״̬
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);
% ����ռ��˶�
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
 
 
figure(2);
grid on;
plot(tTask,stateTask(:,1:numJoints));
hold all;
plot(tTask(1:numJoints),stateTask(1:numJoints),"--");
title("Joint Position vs Reference ");
xlabel("Time (s)")
ylabel("Position (rad)");

