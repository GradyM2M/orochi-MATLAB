%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7���е�۹켣�滮 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/17
% DH: SDH
% �켣�滮������kinovaGen3�������˶�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
% show(robot);
numJoints = numel(homeConfiguration(robot));  % �ؽ�����
endEffector = "EndEffector_Link";  % ĩ��ִ����

%% ��ʼλ��
taskInit = trvec2tform([[0.4 0 0.2]])*axang2tform([1 1 0 pi/2]);

%% �˶�ѧ���
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];
currentRobotJConfig = ik(endEffector, taskInit, weights, robot.homeConfiguration);
currentRobotJConfig = wrapToPi(currentRobotJConfig);

%% ĩ��λ��
taskFinal = trvec2tform([0.2 0.6 0.27])*axang2tform([1 1 0 pi]);  
anglesFinal = rotm2eul(taskFinal(1:3,1:3),'XYZ');
poseFinal = [taskFinal(1:3,4);anglesFinal']; % 6x1 vector for final pose: [x, y, z, phi, theta, psi]

%% ��ʼ��������ײ
collisionHelper = helperManipCollsionsKINOVA(robot);
collisionHelper.ExhaustiveChecking = true;

isMovingObst = false;
helperCreateObstaclesKINOVA;

x0 = [currentRobotJConfig', zeros(1,numJoints)];
helperInitialVisualizerKINOVA;
 
safetyDistance = 0.01; % ���ð�ȫ����

helperDesignNLMPCobjKINOVA;

%% �켣�滮
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


% ���涯��
options = nlmpcmoveopt;
for timestep=1:maxIters
    disp(['Calculating control at timestep ', num2str(timestep)]);
    % �Ż���һ���켣��
    [mv,options,info] = nlmpcmove(nlobj,x0,mv,[],[], options);
    if info.ExitFlag < 0
        disp('Failed to compute a feasible trajectory. Aborting...')
        break;
    end
    % ����״̬����һ�ε�����ʱ��
    x0 = info.Xopt(2,:);
    time = time + nlobj.Ts;
    % ����켣��
    positions(:,timestep+1) = x0(1:numJoints)';
    velocities(:,timestep+1) = x0(numJoints+1:end)';
    accelerations(:,timestep+1) = info.MVopt(2,:)';
    timestamp(timestep+1) = time;
    % ���Ŀ���Ƿ�ﵽ
    helperCheckGoalReachedKINOVA;
    if goalReached
        break;
    end
    % �����ϰ�����
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

%% �Է����л����˵Ĺ켣����п���
initState = [positions(:,1);velocities(:,1)];
targetStates = [positions;velocities;accelerations]';    
[t,robotStates] = ode15s(@(t,state) helperTimeBasedStateInputsKINOVA(motionModel, timestamp, targetStates, t, state), [timestamp(1):visTimeStep:timestamp(end)], initState);

helperFinalVisualizerKINOVA;
