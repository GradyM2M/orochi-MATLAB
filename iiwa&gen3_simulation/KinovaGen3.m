%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7轴机械臂轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/17
% DH: SDH
% 轨迹规划，驱动kinovaGen3机器人运动
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
% show(robot);
numJoints = numel(homeConfiguration(robot));  % 关节数量
endEffector = "EndEffector_Link";  % 末端执行器

%% 初始位子
taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([1 1 0 pi/2]);

%% 运动学逆解
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
currentRobotJConfig = wrapToPi(currentRobotJConfig);

%% 末端位姿
taskFinal = trvec2tform([0.2 0.6 0.27])*axang2tform([1 1 0 pi]);  
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

%% 初始化网格碰撞
collisionHelper = helperManipCollsionsKINOVA(robot);
collisionHelper.ExhaustiveChecking = true;

isMovingObst = false;
helperCreateObstaclesKINOVA;

x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;
 
safetyDistance = 0.01; % 设置安全距离

helperDesignNLMPCobjKINOVA;

%% 轨迹规划
maxIters = 150;
u0 = zeros(1,numJoints);
mv = u0;
time = 0;
goalReached = false;


positions = zeros(numJoints,maxIters);
positions(:,1) = x0(1:numJoints)';
velocities = zeros(numJoints,maxIters);
velocities(:,1) = x0(numJoints+1:end)';
accelerations = zeros(numJoints,maxIters);
accelerations(:,1) = u0';
timestamp = zeros(1,maxIters);
timestamp(:,1) = time;


% 仿真动画
options = nlmpcmoveopt;
for timestep=1:maxIters
    disp(['Calculating control at timestep ', num2str(timestep)]);
    % 优化下一个轨迹点
    [mv,options,info] = nlmpcmove(nlobj,x0,mv,[],[], options);
    if info.ExitFlag < 0
        disp('Failed to compute a feasible trajectory. Aborting...')
        break;
    end
    % 更新状态和下一次迭代的时间
    x0 = info.Xopt(2,:);
    time = time + nlobj.Ts;
    % 保存轨迹点
    positions(:,timestep+1) = x0(1:numJoints)';
    velocities(:,timestep+1) = x0(numJoints+1:end)';
    accelerations(:,timestep+1) = info.MVopt(2,:)';
    timestamp(timestep+1) = time;
    % 检查目标是否达到
    helperCheckGoalReachedKINOVA;
    if goalReached
        break;
    end
    % 更新障碍姿势
    if isMovingObst
        helperUpdateMovingObstaclesKINOVA;
    end
end

tFinal = timestep+1;
positions = positions(:,1:tFinal);
velocities = velocities(:,1:tFinal);
accelerations = accelerations(:,1:tFinal);
timestamp = timestamp(:,1:tFinal);
visTimeStep = 0.1;

motionModel = jointSpaceMotionModel('RigidBodyTree', robot);

%% 对仿真中机器人的轨迹点进行控制
initState = [positions(:,1);velocities(:,1)];
targetStates = [positions;velocities;accelerations]';    
[t,robotStates] = ode15s(@(t,state) helperTimeBasedStateInputsKINOVA(motionModel, timestamp, targetStates, t, state), [timestamp(1):visTimeStep:timestamp(end)], initState);

helperFinalVisualizerKINOVA;
