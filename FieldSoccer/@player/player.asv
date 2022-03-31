classdef player
    %PLAYER Class defining a robot player
    
    %All information contained in the properties is exact and assumes
    %that the player has perfect perception, odometry, and other
    %information provided by the Game Controller. 
    %To simulate actual conditions a special method must be called to
    %add noise to world parameters before passing to other functions
    
    %% public properties
     
    properties
        pos %player position [x,y,a] (player will always think this is in the + direction)
        vel_des %player desired velocity [x,y,a]
        kick %1 if player is attempting a kick, 0 otherwise
        gametime %total game time that has passed
        team_color %player color
        player_number %player number
        role %player role
        pffs %potential field functions for all roles
        imSelection
        frame1
        frame2
        frame3
        frame4
        frame5
        frame6
        frame7
        frame8
        frame9
     
    end

    
    
    %% Private properties
    
    properties (Access = protected)
        num_teammates %number of teammates
        prev_time %ime of last update       
        realtime %if the simulation is running in realtime
        timestep %simulation timestep
        vel %actual velocity
        cfg %local copy of configuration variable
        draw_handle %handle for player drawing
         draw_handle1
          draw_handle2
           draw_handle3
        text_handle %handle for player number
        text_handle2 %handle for player role
        hitbox_handle %handle for player hitbox
        behaviorState %current behavior FSM state
        prev_ball %struct with last ball observation info
        behavior_handle %handle to behavior function
        pos_des %desired position
        world_function_handle %handle to world function
        bh_init %flag if FSM needs to init next behavior state
        local2globalTF %transform from global coordinates to local coordinates  
        dir %direction player is attacking
        radius
    end
    
    %% Constants
    
    properties (Constant)
        
        %roles
        GOALIE = 0;
        ATTACKER = 1;
        DEFENDER = 2;
        SUPPORTER = 3;
        DEFENDER2 = 4;
        NONE = 5;
        
        %states
        SEARCH = 0;
        MOVE = 1;
        KICK = 2;
        APPROACH = 3;
        
        %role names
        roleNames = {'Goalie','Attacker','Defender','Supporter','Defender2'}
    end
   
 
    %% Public methods
    
    methods
        
        %class constructor
        function obj = player(color,pos,num,teammates,cfg,pff_funcs,bh_list)
           
            %defualt values
            obj.pos = [0,0,0];
            obj.vel_des= [0,0,0];
            obj.vel = [0,0,0];
            obj.num_teammates = 0;
            obj.prev_time = tic;
            obj.team_color = 'black';
            obj.player_number = 0;
            obj.realtime = true;
            obj.timestep = 0.1;
            obj.gametime = 0;
            obj.draw_handle = [];
             obj.draw_handle1 = [];
              obj.draw_handle2 = [];
               obj.draw_handle3 = [];
            obj.text_handle = [];
            obj.text_handle2 = [];
            obj.hitbox_handle = [];
            obj.kick = 0;
            obj.cfg = [];
            obj.role = [];
            obj.behaviorState = player.SEARCH;
            obj.prev_ball.time = 0;
            obj.prev_ball.pos = [0,0];
            obj.pos_des = [0,0,0];
            obj.bh_init = true;
            obj.local2globalTF = ones(3,3);
            obj.radius = cfg.player_radius;
           
            obj.imSelection = randi([0 8]);
            obj.frame1 = imread('1.png');
            obj.frame2 = imread('2.png');
            obj.frame3 = imread('3.png');
            obj.frame4 = imread('4.png');
            obj.frame5 = imread('5.png');
            obj.frame6 = imread('6.png');
            obj.frame7 = imread('7.png');
            obj.frame8 = imread('8.png');
            obj.frame9 = imread('9.png');
          





            %add in various arguements
            if nargin >= 1; obj.team_color = color; end
            if nargin >= 2; obj.pos = pos; end
            if nargin >= 3; obj.player_number = num; end
            if nargin >= 4; obj.num_teammates = teammates; end
            if nargin >= 5  
                obj.cfg = cfg;
                obj.timestep = cfg.timestep;
                obj.realtime = cfg.realtime; 
                if strcmp(color,'red')
                    obj.dir = 1;
                    obj.role = cfg.start_roles_red(num);
                else
                    obj.dir = -1;
                    obj.role = cfg.start_roles_blue(num);
                end  
                if obj.cfg.world_random_on
                    obj.world_function_handle = @get_world_random;
                else
                    obj.world_function_handle = @get_world_exact;
                end
            end    
            if nargin >= 6; obj.pffs = pff_funcs; end 
            if nargin >= 7; obj.behavior_handle = bh_list; end
        end
        
        
        %Update function should be called on every loop. 
        function obj = update(obj,w)
            
            %'observe' information from the world/teammates
            world = obj.world_function_handle(w,obj.team_color,obj.player_number);
            
            %update role (computed centrally to reduce time)
            obj.role = world.cur_player.role;
            
            %make behavioral decisions based on observed data
            obj = behavior_advancedFSM(obj,world);
           
            %update kinematics based on desired velocity
            obj = update_pos(obj);
        end
            
        
        %get player current velocity
        function v = get_vel(obj)
            v = obj.vel;
        end
        
        %sets velocity to 0
        function obj = SetZeroVel(obj)
            obj.vel = [0,0,0];
            obj.vel_des = [0,0,0];
        end
        
        
        %overrides velocity - for collisions ONLY
        function obj = vel_override(obj,v)
            obj.vel = v;
        end
    
        %figure out which behavior handle should be used for a given role
        function id = get_bh_id(obj)
            if strcmp(obj.team_color,'red')
                id = find(obj.cfg.start_roles_red == obj.role);
            else
                id = find(obj.cfg.start_roles_blue == obj.role);
            end
        end
        
        
        %draws player on specified axes
        function obj = draw_player(obj,ax)

%             cosd = d*cos(obj.pos(3));
%             sind = d*sin(obj.pos(3));
%             player = imread("walk.png");
           delete(obj.draw_handle3);
           delete(obj.draw_handle1);
           delete(obj.draw_handle2);

%            obj.draw_handle = player;
          
%%

X = interp1([-5,5],[0.1825,0.8125],obj.pos(1)); %
Y = interp1([-4,4],[0.07,0.885],obj.pos(2));%
angle = obj.pos(3)*180/pi+270;

% % X = 0.1825; %most left x
% X = 0.5; %most right x
% % Y = 0.07; %most btm y
% Y = 0.5; %most btm y
% angle = 0;

% angle =   90;
if obj.imSelection == 0
img = imrotate(obj.frame1,angle,'crop');
elseif obj.imSelection == 1
img = imrotate(obj.frame2,angle,'crop');
elseif obj.imSelection == 2
img = imrotate(obj.frame3,angle,'crop');
elseif obj.imSelection == 3
img = imrotate(obj.frame4,angle,'crop');
elseif obj.imSelection == 4
img = imrotate(obj.frame5,angle,'crop');
elseif obj.imSelection == 5
img = imrotate(obj.frame6,angle,'crop');
elseif obj.imSelection == 6
img = imrotate(obj.frame7,angle,'crop');
elseif obj.imSelection == 7
img = imrotate(obj.frame8,angle,'crop');
elseif obj.imSelection == 8
img = imrotate(obj.frame9,angle,'crop');
end
frame = img;
alpha1 = img(:,:,1);
alpha2 = img(:,:,2);
alpha3 = img(:,:,3);

axes('Position',[X Y 0.04 0.08]);
obj.draw_handle1 = image(frame, 'AlphaData', alpha1);hold on;
obj.draw_handle2 = image(frame, 'AlphaData', alpha2);hold on;
obj.draw_handle3 = image(frame, 'AlphaData', alpha3);hold on;

%%
%             temp = imread('walk.png');
%             %alpha1 = imresize(temp,0.001,'bicubic');
%             x = interp1([-5,5],[0.106,0.88],obj.pos(1));
%             y = interp1([-4,4],[0.055,0.975],obj.pos(2));
%             %x= obj.pos(1);y = obj.pos(2);
%             axes('Position',[x y 0.04 0.08]);
%             %axes('Position',[obj.pos(1) obj.pos(2) 0.04 0.08]);
%             %obj.draw_handle = image(ax, 'AlphaData', image1);
%             frame = temp;
%             alpha1 = temp(:,:,1);
%             %alpha2 = temp(:,:,2);
%             %alpha3= temp(:,:,3);
%             image(frame, 'AlphaData', alpha1);
%             %image(frame, 'AlphaData', alpha1);
%             %hold on;
%             %image(frame, 'AlphaData', alpha2);
%             %hold on;
%             %image(frame, 'AlphaData', alpha3);
%             %hold on
%             angle = obj.pos(3);
%             img = imrotate(frame,angle,'crop');
%             obj.draw_handle = image(img, 'AlphaData', alpha1);
%            
%             
% 
%             box on
%             axis off;
           %%
            %obj.draw_handle = (rectangle(ax,'Position',[px py d d],'Curvature',[1,1],'FaceColor',obj.team_color,'EdgeColor','none'));
           
            % obj.draw_handle = line([obj.pos(1)-cosd,obj.pos(1)-cosd],[obj.pos(2)-sind,obj.pos(2)+sind],'black','b');
%           delete(obj.draw_handle1)
%             delete(obj.draw_handle2)
%             delete(obj.draw_handle3)
%             delete(obj.text_handle)
%             delete(obj.text_handle2)
% 
%             temp = imread('walk.png');
%             
%             [X,Y] = obj.get_pose_plot();
%             angle = obj.pos(3)*180/pi;
% 
%             img = imrotate(temp,angle,'crop');
% 
%             frame = img;
%             alpha1 = img(:,:,1);
%             alpha2 = img(:,:,2);
%             alpha3 = img(:,:,3);
% 
%             x = interp1([-5,5],[0.106,0.88],X);
%             y = interp1([-4,4],[0.055,0.975],Y);
% 
% 
%             obj.draw_handle1 = image(frame, 'AlphaData', alpha1);hold on;
%             obj.draw_handle2 = image(frame, 'AlphaData', alpha2);hold on;
%             obj.draw_handle3 = image(frame, 'AlphaData', alpha3);hold on;           
         box on;
         axis off;

    


        end
        
        
        %draws player hitbox for debugging
        function obj = draw_player_hitbox(obj,ax,cfg)
            d = cfg.player_hitbox_radius*2;
            px = obj.pos(1) - cfg.player_hitbox_radius;
            py = obj.pos(2) - cfg.player_hitbox_radius;
            
            delete(obj.hitbox_handle);
            obj.hitbox_handle = rectangle(ax,'Position',[px py d d],...
                'Curvature',[1,1],'FaceColor','none','EdgeColor','black');            
        end
        
    end
    
    
    %% static utility methods
    
    methods(Access = protected, Static)
    
        %Gets 2D tranformation matrix (3x3) for xya vector
        function T = xya2transform(xya)
            T = zeros(3);
            T(1,3) = xya(1);
            T(2,3) = xya(2);
            a = xya(3);
            c = cos(a);
            s = sin(a);
            T(1,1) = c;
            T(2,2) = c;
            T(1,2) = -s;
            T(2,1) = s;
            T(3,3) = 1;
        end
        
        %Gets xya vector from 2D transformation matrix (3x3)
        function xya = transform2xya(T)
            xya(1) = T(1,3);
            xya(2) = T(2,3);
            xya(3) = atan2(T(2,1),T(1,1));            
        end
            
    end
 
    
    %% Protected/private methods

    methods (Access = protected)        
        
        %gets the pose of the robot in a formatted form to plot easily
        function [X,Y] = get_pose_plot(obj)
   
            %default player marker dimensions
            a = 0.05;
            b = 0.15;
            Y0 = [-a 0 a];
            X0 = [-b/2, b/2, -b/2];
            
            %transform to current coordinates
            T = player.xya2transform(obj.pos);
            P = [X0;Y0;ones(1,3)];
            P1 = T*P;
            X = P1(1,:);
            Y = P1(2,:);
   
        end
        
        %updates postion and velocity based on current position, desired
        %velocity, and time. It will do
        %transformations to the player location based on current velocity
        function obj = update_pos(obj)
            
             if obj.realtime
                
                %get dt since last update
                dt = toc(obj.prev_time);
                obj.prev_time = tic;
                
            else
                %if we are not realtime, just use defualt timestep
                dt = obj.timestep;
            end
            
            %perform acceleration if not at desired velocity
            dv = obj.vel_des - obj.vel;
            if any(dv ~= 0)
                
                %compute linear velocity direction
                dv_lin = dv(1:2);
                if norm(dv_lin) > 0
                    linear_direction  = dv_lin/norm(dv_lin);
                else
                    linear_direction = [0,0];
                end
                
                %do linear and angular acceleration
                obj.vel(1:2) = obj.vel(1:2) + linear_direction.*obj.cfg.player_accelLin * dt;
                obj.vel(3) = obj.vel(3) + sign(obj.vel_des(3))*obj.cfg.player_accelAng*dt;
                
                %cap velocites at desired to prevent wobble (and avoid expensive
                %controller)
                inc = dv>0;
                dec = dv<0;
                obj.vel(inc) = sign(obj.vel_des(inc)).*min(abs(obj.vel(inc)),abs(obj.vel_des(inc)));
                obj.vel(dec) = sign(obj.vel_des(dec)).*max(abs(obj.vel(dec)),abs(obj.vel_des(dec)));                
                
                %cap velocities at max in each direction
                if obj.vel(1) < 0
                    obj.vel(1) = max(obj.vel(1),obj.cfg.player_MaxLinVelX(2));
                else
                    obj.vel(1) = min(obj.vel(1),obj.cfg.player_MaxLinVelX(1));
                end
                obj.vel(2) = sign(obj.vel(2)) * min(abs(obj.vel(2)),obj.cfg.player_MaxLinVelY);
                obj.vel(3) = sign(obj.vel(3)) * min(abs(obj.vel(3)),obj.cfg.player_MaxAngVel);
                
               
                
            end        
            
            %position change in local coorinate system
            dp_local = player.xya2transform(obj.vel*dt);
            
            %conversion to global coordinates
            obj.local2globalTF = player.xya2transform([0,0,obj.pos(3)]);
            dp_global =  obj.local2globalTF*dp_local;
            
            %update global position
            tmp = player.xya2transform(obj.pos);
            obj.pos = player.transform2xya(tmp+dp_global);
             
            % % % %update imSelection
if obj.imSelection == 0
obj.imSelection = 1;
elseif obj.imSelection == 1
obj.imSelection = 2;
elseif obj.imSelection == 2
obj.imSelection = 3;
elseif obj.imSelection == 3
obj.imSelection = 4;
elseif obj.imSelection == 4
obj.imSelection = 5;
elseif obj.imSelection == 5
obj.imSelection = 6;
elseif obj.imSelection == 6
obj.imSelection = 7;
elseif obj.imSelection == 7
obj.imSelection = 8;
elseif obj.imSelection == 8
obj.imSelection = 0;
end
            %update game time
            obj.gametime = obj.gametime + dt;
        end
                
    end
    
end

