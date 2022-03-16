function cfg = ParameterConfiguration()
%initialize configuration variables
%initialize configuraiton variable
cfg = [];
    
cfg.num_players_red = 4;
cfg.num_players_blue = 4;

%timing
cfg.timestep = 0.05;
cfg.halflength = 300; %seconds

%number of players
cfg.num_players = cfg.num_players_red + cfg.num_players_blue;

%starting positions
cgf.ball_position = [0,0];
% cfg.start_pos(2,:) = [-1,0,0];
% cfg.start_pos(3,:) = [-2,-1,0];
% cfg.start_pos(4,:) = [-2,1,0];
% cfg.start_pos(5,:) = [-3,-0.5,0];
% cfg.start_pos(1,:) = [-3,0.5,0];
% 
% cfg.start_pos(7,:) = [1,0,pi];
% cfg.start_pos(8,:) = [2,1,pi];
% cfg.start_pos(9,:) = [2,-1,pi];
% cfg.start_pos(10,:) = [3,-0.5,pi];
% cfg.start_pos(6,:) = [3,0.5,pi];

%ball parameters
cfg.ball_radius = 0.0715;       %m
cfg.ball_mass = 0.3;

%field
cfg.field_length= 4.5; %m
cfg.field_width = 3; %m
cfg.field_length_max = 1.1*cfg.field_length;
cfg.field_width_max = 1.1*cfg.field_width;
cfg.line_thickness = 0.05;
cfg.goal_posts = [4.5,-0.8; 4.5,0.8; -4.5,-0.8; -4.5,0.8];
cfg.goal_depth = 0.25;
cfg.spots = [-3.2,0; 3.2,0];
cfg.spot_size = 0.1;
cfg.penalty_corners = [-3.9,1.1; -3.9,-1.1; 3.9,1.1; 3.9,-1.1];
cfg.penaltyY = cfg.penalty_corners(1,2)-cfg.penalty_corners(2,2);
cfg.penaltyX = cfg.field_length - cfg.penalty_corners(3,1);
cfg.circle_radius = 0.75;
cfg.oobLineY = cfg.field_width-0.25;
cfg.oobLineX = cfg.field_length - 0.5;
cfg.stepSize_canvas = 0.01;
end




