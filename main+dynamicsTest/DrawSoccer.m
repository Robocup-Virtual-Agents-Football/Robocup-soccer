function [] = DrawSoccer(ball_radius, ball_position) %, headingAngle)
%DRAWSOCCER - Draws a representation of the robot and its orientation

% Inputs: 
%   robot_position(1)         x-coordinate of centre of soccer ball 
%   robot_position(2)         y-coordinate of centre of soccer ball 
%   headingAngle              current heading angle, in radians

% Outputs: none (plot to current figure)


% Created by         : Jia Wei Tan

ax = gca;

%% Calculate end points of the line
% Line is 0.02m larger than the robot
% lineEnd_x = cos(headingAngle) * (ball_radius + 0.02);
% lineEnd_y = sin(headingAngle) * (ball_radius + 0.02);

%% Draw Ball
viscircles(ball_position,ball_radius,'Color','black');
viscircles(ball_position,ball_radius-0.02,'Color','white');
viscircles(ball_position,ball_radius-0.04,'Color','white');
viscircles(ball_position,ball_radius-0.06,'Color','white');

% Draw line from center of robot in the direction of the current heading
% line([ball_position(1), (lineEnd_y + ball_position(1))], [ball_position(2), (lineEnd_x + ball_position(2))], 'color', 'r');

end