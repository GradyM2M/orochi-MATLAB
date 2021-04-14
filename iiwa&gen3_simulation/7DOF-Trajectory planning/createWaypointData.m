%% 导入机器人模型
load gen3
load gen3positions
eeName = 'Gripper';
numJoints = numel(gen3.homeConfiguration);
ikInitGuess = gen3.homeConfiguration;

%% Maximum number of waypoints (for Simulink)
maxWaypoints = 20;

% Positions (X Y Z) 续一秒
waypoints = toolPositionHome' + ... 
            [0 0.3 0.3 ; 0 0.5 0.3; 0 0.4 0.3 ;0 0.4 0.4;0 0.4 0.2;....
           -0.1 0.4 0.2 ;-0.1 0.2 0.5 ;0 0.2 0.5 ;0 0.2 0.1;-0.1 0.2 0.1;...
            -0.1 -0.1 0.4 ;0 -0.1 0.4 ;0 0 0.5 ;0 0.15 0.4 ;0 0 0.3 ;...
             0 -0.1 0.2; 0 0.05 0.1 ; 0 0.1 0.2;]';
         
%% Euler Angles (Z Y X) relative to the home orientation       
orientations = [0     0    0;
                pi/8  0    0; 
                0    pi/2  0;
               -pi/8  0    0;
                0     0    0]';   
            
%% Array of waypoint times
waypointTimes = 0:4:68;

%% Trajectory sample time
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Additional parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1 *[ 0  0  0;
                      0  0  0;
                      0 0  0;
                      0  0  0;
                      0  0  0;
                      -1  0  0;
                      1  0  0;
                      0  0  0;
                      0  0  0;
                      -1 0  0; 
                      1  0  0;
                      1  0  0;
                      0  0.5  0.5;
                      0  -0.5  -0.5;
                      0  -0.5  -0.5;
                      0  -0.5  -0.5;
                      0  0.5  0.5;
                      0  -0.5  -0.5;
                      ]';

%% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

%% Acceleration times (trapezoidal only)
waypointAccelTimes = diff(waypointTimes)/4;
