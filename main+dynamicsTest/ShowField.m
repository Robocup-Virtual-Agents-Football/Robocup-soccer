function [imSelectionOut] = ShowField(ball_position, robot_position, headingAngle, robot_state, imSelection)
%SHOWFIELD - Draws a representation of the ball, robots and their orientation

% Inputs: 
%   ball_position             coordinates of ball
%   robot_position(n,:)       coordinates of Robot-n       
%   headingAngle(n)           current heading angle of Robot-n, in radians
%   robot_state(n)            state of robot (not moving = 0, moving = 1)
%   imSelection               select image (0 = static, 1 = frame1, 2 = frame2)

% Outputs: 
%   imSelectionOut            previous image

cfg = ParameterConfiguration;
ax = gca;


%load images
% staticIm = imread('static2.png');
% frame1 = imread('pos1.png');
% frame2 = imread('pos2.png');
frame1 = imread('1.png');
frame2 = imread('2.png');
frame3 = imread('3.png');
frame4 = imread('4.png');
frame5 = imread('5.png');
frame6 = imread('6.png');
frame7 = imread('7.png');
frame8 = imread('8.png');
frame9 = imread('9.png');

%green field
rectangle(ax ,'Position',[-cfg.field_length_max,-cfg.field_width_max,2*cfg.field_length_max,2*cfg.field_width_max],...
    'FaceColor','#32CD32','EdgeColor','none');

%boundary lines
rectangle(ax ,'Position',[-cfg.field_length,-cfg.field_width,2*cfg.field_length,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[-cfg.field_length,cfg.field_width,2*cfg.field_length,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[-cfg.field_length,-cfg.field_width,cfg.line_thickness,2*cfg.field_width],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.field_length,-cfg.field_width,cfg.line_thickness,2*cfg.field_width+cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
%center line
rectangle(ax ,'Position',[-cfg.line_thickness/2,-cfg.field_width,cfg.line_thickness,2*cfg.field_width],...
    'FaceColor','white','EdgeColor','none')
%penalty box lines
rectangle(ax ,'Position',[cfg.penalty_corners(2,1),cfg.penalty_corners(2,2),cfg.line_thickness,cfg.penaltyY],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.penalty_corners(4,1),cfg.penalty_corners(4,2),cfg.line_thickness,cfg.penaltyY],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.penalty_corners(1,1)-cfg.penaltyX,cfg.penalty_corners(1,2),cfg.penaltyX+cfg.line_thickness,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.penalty_corners(2,1)-cfg.penaltyX,cfg.penalty_corners(2,2),cfg.penaltyX+cfg.line_thickness,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.penalty_corners(3,1),cfg.penalty_corners(3,2),cfg.penaltyX+cfg.line_thickness,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.penalty_corners(4,1),cfg.penalty_corners(4,2),cfg.penaltyX+cfg.line_thickness,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
%spots
rectangle(ax ,'Position',[cfg.spots(1,1)-cfg.spot_size/2,cfg.spots(1,2)-cfg.line_thickness/2,cfg.spot_size,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.spots(1,1)-cfg.line_thickness/2,cfg.spots(1,2)-cfg.spot_size/2,cfg.line_thickness,cfg.spot_size],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.spots(2,1)-cfg.spot_size/2,cfg.spots(2,2)-cfg.line_thickness/2,cfg.spot_size,cfg.line_thickness],...
    'FaceColor','white','EdgeColor','none')
rectangle(ax ,'Position',[cfg.spots(2,1)-cfg.line_thickness/2,cfg.spots(2,2)-cfg.spot_size/2,cfg.line_thickness,cfg.spot_size],...
    'FaceColor','white','EdgeColor','none')
%circle
rectangle(ax ,'Position',[-cfg.circle_radius,-cfg.circle_radius,2*cfg.circle_radius,2*cfg.circle_radius],...
    'FaceColor','none','EdgeColor','white','Curvature',[1,1],'LineWidth',4)
%goals
rectangle(ax ,'Position',[cfg.goal_posts(1,1)+cfg.line_thickness/2,cfg.goal_posts(1,2),cfg.goal_depth,-cfg.goal_posts(1,2)+cfg.goal_posts(2,2)],...
    'FaceColor','blue','EdgeColor','white','LineWidth',5)
rectangle(ax ,'Position',[cfg.goal_posts(3,1)-cfg.goal_depth+cfg.line_thickness/2,cfg.goal_posts(3,2),cfg.goal_depth,-cfg.goal_posts(3,2)+cfg.goal_posts(4,2)],...
    'FaceColor','red','EdgeColor','white','LineWidth',5)

%return figure
fig = gcf;

% DrawSoccer(ball_position, robot_position, headingAngle);
%% Draw Ball
viscircles(ball_position,cfg.ball_radius,'Color','black');
viscircles(ball_position,cfg.ball_radius-0.02,'Color','white');
viscircles(ball_position,cfg.ball_radius-0.04,'Color','white');
viscircles(ball_position,cfg.ball_radius-0.06,'Color','white');

%Setup window
axis tight
axis equal
set(gcf, 'Position', get(0, 'Screensize'));
xlabel('X')
ylabel('Y')

%% Draw Robot
for i = 1:(cfg.num_players)

%robot heading angle
angle = headingAngle(i)*180/pi;

if robot_state(i) == 0
    img = imrotate(frame1,angle,'crop');
end

if robot_state(i) == 1
    if imSelection(i) == 0
        img = imrotate(frame1,angle,'crop');
        imSelection(i) = 1;
    elseif imSelection(i) == 1
        img = imrotate(frame2,angle,'crop');
        imSelection(i) = 2;
    elseif imSelection(i) == 2
        img = imrotate(frame3,angle,'crop');
        imSelection(i) = 3;
    elseif imSelection(i) == 3
        img = imrotate(frame4,angle,'crop');
        imSelection(i) = 4;
    elseif imSelection(i) == 4
        img = imrotate(frame5,angle,'crop');
        imSelection(i) = 5;
    elseif imSelection(i) == 5
        img = imrotate(frame6,angle,'crop');
        imSelection(i) = 6;
    elseif imSelection(i) == 6
        img = imrotate(frame7,angle,'crop');
        imSelection(i) = 7;
    elseif imSelection(i) == 7
        img = imrotate(frame8,angle,'crop');
        imSelection(i) = 8;
    elseif imSelection(i) == 8
        img = imrotate(frame9,angle,'crop');
        imSelection(i) = 0;
    end
end

frame = img;
alpha1 = img(:,:,1);
alpha2 = img(:,:,2);
alpha3 = img(:,:,3);

% Robot Coordinates
% mapping -5 to 5 ---> 0.106 to 0.88, -4 to 4 ---> 0.055-0.975
x = interp1([-5,5],[0.106,0.88],robot_position(i,1));
y = interp1([-4,4],[0.055,0.975],robot_position(i,2));

axes('Position',[x y 0.04 0.08]);
image(frame, 'AlphaData', alpha1);
hold on;
image(frame, 'AlphaData', alpha2);
hold on;
image(frame, 'AlphaData', alpha3);
hold on;

box on;
axis off;

end

imSelectionOut = imSelection;

end

