function [] = DrawSoccer(ball_position, robot_position, headingAngle)
%DRAWSOCCER - Draws a representation of the robot and its orientation

% Inputs: 
%   ball_radius               radius of ball
%   ball_position             coordinates of ball
%   robot_position(n,:)       coordinates of Robot-n       
%   headingAngle(n)           current heading angle of Robot-n, in radians

% Outputs: none (plot to current figure)

% Created by         : Jia Wei Tan

ax = gca;
cfg = ParameterConfiguration();

%% Draw Ball
viscircles(ball_position,cfg.ball_radius,'Color','black');
viscircles(ball_position,cfg.ball_radius-0.02,'Color','white');
viscircles(ball_position,cfg.ball_radius-0.04,'Color','white');
viscircles(ball_position,cfg.ball_radius-0.06,'Color','white');

% Draw line from center of robot in the direction of the current heading
% line([ball_position(1), (lineEnd_y + ball_position(1))], [ball_position(2), (lineEnd_x + ball_position(2))], 'color', 'r');

%% Draw Robot
for i = 1:(cfg.num_players)
    viscircles(robot_position(i,:),0.15,'Color','black');

    %% Calculate end points of the line
    % Line is 0.01m larger than the robot
    lineEnd_x = cos(headingAngle(i)) * (0.15 + 0.03);
    lineEnd_y = sin(headingAngle(i)) * (0.15 + 0.03);

    line([robot_position(i,1), (lineEnd_y + robot_position(i,1))],...
        [robot_position(i,2), (lineEnd_x +robot_position(i,2))], 'color', 'b');


end


end