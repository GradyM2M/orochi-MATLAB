%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 7轴机械臂轨迹规划 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Nym
% Date: 2020/02/16
% DH: SDH
% 驱动KUKA R820机器人抓取杯子(连杆模型——Orochi臂)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc

%% 1.读取机器人模型

lbr = importrobot('Orochi.urdf'); 
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';

%% 2.建立杯子模型

cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [-0.5, 0.5, cupHeight/2];
body = rigidBody('cupFrame');
setFixedTransform(body.Joint, trvec2tform(cupPosition))
addBody(lbr, body, lbr.BaseName);

%% 3.设置参数和碰撞极限距离

numWaypoints = 10;
q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, numWaypoints, 1);
gik = generalizedInverseKinematics('RigidBodyTree',lbr,'ConstraintInputs',...
    {'cartesian','position','aiming','orientation','joint'});

heightAboveTable = constraintCartesianBounds(gripper);
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.05, inf];
distanceFromCup = constraintPositionTarget('cupFrame');
distanceFromCup.ReferenceBody = gripper;
distanceFromCup.PositionTolerance = 0.005;

alignWithCup = constraintAiming('iiwa_link_ee');
alignWithCup.TargetPoint = [0, 0, 100];
limitJointChange = constraintJointBounds(lbr);
fixOrientation = constraintOrientationTarget(gripper);
fixOrientation.OrientationTolerance = deg2rad(1);

intermediateDistance = 0.3;

limitJointChange.Weights = zeros(size(limitJointChange.Weights));
fixOrientation.Weights = 0;

distanceFromCup.TargetPosition = [0,0,intermediateDistance];
[qWaypoints(2,:),solutionInfo] = gik(q0, heightAboveTable, ...
                       distanceFromCup, alignWithCup, fixOrientation, ...
                       limitJointChange);

limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;
alignWithCup.Weights = 0;
fixOrientation.TargetOrientation = ...
    tform2quat(getTransform(lbr,qWaypoints(2,:),gripper));

finalDistanceFromCup = 0.05;
distanceFromCupValues = linspace(intermediateDistance,finalDistanceFromCup,numWaypoints-1);

maxJointChange = deg2rad(10);


%% 4.更新当前位置

for k = 3:numWaypoints
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k-1);
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         heightAboveTable, ...
                                         distanceFromCup, alignWithCup, ...
                                         fixOrientation, limitJointChange);
end

framerate = 15;
r = rateControl(framerate);
tFinal = 10;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
numFrames = tFinal*framerate;
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';

gripperPosition = zeros(numFrames,3);
for k = 1:numFrames
    gripperPosition(k,:) = tform2trvec(getTransform(lbr,qInterp(k,:),gripper));
end

%% 5.画出运行轨迹

figure;
show(lbr, qWaypoints(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
p = plot3(gripperPosition(1,1), gripperPosition(1,2), gripperPosition(1,3));

hold on
for k = 1:size(qInterp,1)
    show(lbr, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k,1);
    p.YData(k) = gripperPosition(k,2);
    p.ZData(k) = gripperPosition(k,3);
    waitfor(r);
end
hold off
save('lbr_trajectory.mat', 'tWaypoints', 'qWaypoints');
