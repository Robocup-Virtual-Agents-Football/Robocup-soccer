%% Preamble
close all;
clear all;
clc;

%% initialize variables
cfg = ParameterConfiguration();

% Time
stepSize_time = cfg.timestep;                            

%% TEMPORARY
simulationTime_total = 3;  
x = 0;
y = 0;

force = [7 7];
vel_ball = [0 0];
ball_position = [0 0];
robot_position = [-3 2; -3 1; -3 0; -3 -1; 3 2; 3 1; 3 0; 3 -1];
headingAngle = [1 1 1 1 1 1 1 1];
robot_state = [1 1 1 1 1 1 1 1];
imSelection = [0 0 0 0 0 0 0 0];

%force = [0 0];
%vel_ball = [1 1];
%ball_position = [0 0];




%% Environment
canvasSize_horizontal = cfg.field_length_max;
canvasSize_vertical   = cfg.field_width_max;
stepSize_canvas       = cfg.stepSize_canvas;



%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
% [wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix); 
% [wall_2, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);

%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
time = 0;

% Run simulation
for timeStep = 1:timeSteps_total

    time(timeStep + 1)    = timeStep * stepSize_time;

%     [force_new vel_ball_new] = BallDynamicsTest(force(timeStep,:), vel_ball(timeStep,:), ball_position(timeStep,:));
%     ball_position_new = ball_position(timeStep,:) + 0.5*(vel_ball_new +vel_ball(timeStep,:))*cfg.timestep;
% 
%     vel_ball(timeStep+1,:) = vel_ball_new;
%     force(timeStep+1,:) = force_new;
%     ball_position(timeStep+1,:) = ball_position_new;

%% Draw Goal


%% Draw Field

imSelection = ShowField(ball_position, robot_position, headingAngle, robot_state, imSelection);
figure(1); clf; hold on; grid on; 

xlabel('y, m'); ylabel('x, m');

headingAngle = headingAngle;

end

%% Plot

% figure(2); hold on; grid on;
% plot(force);
% title('force against time')
% % 
% figure(3); hold on; grid on;
% plot(vel_ball);
% title('velocity against time')




