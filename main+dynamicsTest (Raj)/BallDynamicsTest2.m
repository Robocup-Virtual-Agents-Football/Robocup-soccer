% function force  = fcn(vel_ball, robot_position, feet_state, strategy, ball_position)
%--------------------|  1x2  |----|  Nx2  |-------| Nx1 |----| Nx1 |----| 1x2 |---
function [force_out, vel_out, pos_out, ballStopped] = BallDynamicsTest2(force_arr, vel_ball_arr, pos_ball_arr, ballStopped)
 
cfg = ParameterConfiguration;

%% Initialize variables
% distance=zeros(2,1);
f_friction = 0;

% robot_no = ;               %no. of robots
leg_mass = 1.201;          %kg
ball_mass = 0.3;           %kg
delta_t = cfg.timestep;                      %time
% max_reach = ;              %max reach of robot feet
mu = 0.4;                  %friction coeficient (b/w 0.37 and 0.62)
g = 9.81;                  %gravity
C_d = 0.47;                %drag coefficient (ball is assumed to be spherical at all times)
A = 2*pi*0.0715^2;         %cross sectional area of ball
rho = 1.225;               %air density
%e= ;                       %coefficient of restitution

%% Test
    
    if ballStopped == true
        force_out = [0, 0];
        vel_out = [0, 0];
        pos_out = pos_ball_arr(end);

    else
        vel_ball = vel_ball_arr(end);
        pos_ball = pos_ball_arr(end);
        direction = vel_ball/norm(vel_ball);
        
        f_friction = mu*ball_mass*g*direction;                              %ff = uN
        f_drag = 0; % 0.5*rho*norm(vel_ball)^2*C_d*A*direction;        %a = pV^2C_dA/2m
        
    % 
        force = force_arr(end, :);
        force_out = force; % - f_friction - f_drag;
    
        v_n = vel_ball;
        s_n = pos_ball;
    
        a_n_plus_1 = (force_out/ball_mass) - 0.5 * exp(size(force_arr, 1) / norm(force_out));
        v_n_plus_1 = v_n + a_n_plus_1 * delta_t;
        s_n_plus_1 = s_n + v_n * delta_t + 0.5 * a_n_plus_1 * delta_t^2;
    
        vel_out = v_n_plus_1;
        pos_out = s_n_plus_1;
        
%         if dot(vel_ball_arr(end, :), vel_out) <= 0
%             force_out = [0, 0];
%             vel_out = [0, 0];
%             ballStopped = true;
%         end

        if size(vel_ball_arr, 1) >= 2
            if (norm(vel_ball_arr(end, :)) < norm(vel_ball_arr(end-1, :)) && norm(vel_ball_arr(end, :)) < norm(vel_out))
                min_vel = norm(vel_ball_arr(end, :))
                force_out = [0, 0];
                vel_out = [0, 0];
                ballStopped = true;
            end
        end

%         if norm(vel_out) <= 1
%             ballStopped = ballStopped + 1;
%         end
        norm(vel_out)
   
    end
end
      

% Goal scored
%if ((ball_position(1,1)<=0) && (ball_position(2,1)>41) && (ball_position(2,1)<59))
%    goal=1; 
%end
%
%if ((ball_position(1,1)>=200) && (ball_position(2,1)>41) && (ball_position(2,1)<59))
%    goal=1; 
%end