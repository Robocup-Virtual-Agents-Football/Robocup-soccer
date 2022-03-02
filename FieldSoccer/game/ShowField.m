function [ fig, stats_handles, ax] = ShowField()
%SHOWFIELD Draws soccer field
%   Reterusn fig which is a handle to current figure containing the field

init = FullRunConfig(1);
cfg = init;
%prep figure window
ax = gca;

%green field
rectangle(ax ,'Position',[-cfg.field_length_max,-cfg.field_width_max,2*cfg.field_length_max,2*cfg.field_width_max],...
    'FaceColor','green','EdgeColor','none');

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

%prep stats
stats_handles.label = uicontrol(fig,'Style','text','String','GAME INFO ',...
    'Units','normalized','Position',[0.02, 0.88,0.12,0.05],...
    'FontSize',20);
stats_handles.elapsed_time = uicontrol(fig,'Style','text','String','Elapsed time: ',...
    'Units','normalized','Position',[0.02, 0.85,0.12,0.05],...
    'FontSize',15);
stats_handles.game_time = uicontrol(fig,'Style','text','String','Game time: ',...
    'Units','normalized','Position',[0.02, 0.82,0.12,0.05],...
    'FontSize',15);
stats_handles.game_speed = uicontrol(fig,'Style','text','String','Game speed: ',...
    'Units','normalized','Position',[0.02, 0.79,0.12,0.05],...
    'FontSize',15);
stats_handles.fps = uicontrol(fig,'Style','text','String','FPS: ',...
    'Units','normalized','Position',[0.02, 0.76,0.12,0.05],...
    'FontSize',15);
stats_handles.score1 = uicontrol(fig,'Style','text','String','Red  Blue ',...
    'Units','normalized','Position',[0.85, 0.88,0.12,0.05],...
    'FontSize',20);
stats_handles.score2 = uicontrol(fig,'Style','text','String','Score: ',...
    'Units','normalized','Position',[0.85, 0.85,0.12,0.05],...
    'FontSize',18);

%Setup window
axis tight
axis equal
set(gcf, 'Position', get(0, 'Screensize'));
xlabel('X')
ylabel('Y')

end

