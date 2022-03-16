% function force  = fcn(vel_ball, robot_position, feet_state, strategy, ball_position)
%--------------------|  1x2  |----|  Nx2  |-------| Nx1 |----| Nx1 |----| 1x2 |---
function [force_out vel_out] = BallDynamics(force, vel_ball, ball_position)
 
cfg = ParameterConfiguration;

%% Initialize variables
% distance=zeros(2,1);
f_friction = 0;

% robot_no = ;               %no. of robots
leg_mass = 1.201;          %kg
ball_mass = 0.3;           %kg
t = cfg.timestep;                      %time
% max_reach = ;              %max reach of robot feet
mu = 0.4;                  %friction coeficient (b/w 0.37 and 0.62)
g = 9.81;                  %gravity
C_d = 0.47;                %drag coefficient (ball is assumed to be spherical at all times)
A = 2*pi*0.0715^2;         %cross sectional area of ball
rho = 1.225;               %air density
%e= ;                       %coefficient of restitution


%% Main Code

% % Compute distance b/w ball and robo
% d_robot_ball = ball_position-robot_position;        %vector robot to ball
% d_robot_ball_mag = vecnorm(d_robot_ball,p,2);       %magnitude (distance)
% direction = d_robot_ball/d_robot_ball_mag;          %direction vector
% 
% max_reach_mat = max_reach*ones(robot_no,1);         %creates nx1 matrix for max_reach
% leg_mass_mat = leg_mass*ones(robot_no,1);           %creates nx1 matrix for leg_mass
% out = d_robot_ball_mag <= max_reach;                %outputs logic array eg: [1; 0; 0...]
% if all(out(:)==0)
%     %if pos_z == 0
%     decel_friction = mu*g*direction;                                        %ff = uN
%     decel_drag = 0.5*rho*norm(vel_ball)^2*C_d*A/ball_mass*direction;        %a = pV^2C_dA/2m
% 
%     decel_tot = decel_drag + decel_friction;
% 
%     vel_ball_final = vel_ball - decel_tot*t;             %v = u +at
%     force = ball_mass*(vel_ball_final-vel_ball)/t;       %f = m(v-u)/t
%     
% else
%     d_robot_ball_mag = d_robot_ball_mag.*out;            %keeps only the values of robots that are within range
%     d_feet_ball = %<__>.*out;                            %equation that relates feet_state and distance
% 
%     if d_feet_ball == d_robot_ball_mag
%         scaling_factor = 0.2*strategy.*out;              %alters instantaneous velocity 
%                                                          %Long pass= 5 | Corner Kick= 4 | Penalty Kick= 3 | Free Kick= 2 | Short pass= 1 | Stop= 0
%         vel_feet = %<__>.*scaling_factor*direction;      %equation that relates instantaneous velocity and feet state, *includes all feets in contact with ball
% 
%         %inelastic collision
%         vel_ball_final = (sum(leg_mass_mat.*vel_feet)+ball_mass*vel_ball)/(sum(leg_mass_mat.*out)+ball_mass);   %vf = (m1v1+m2v2+...)/(m1+m2+...)
% 
%         %inelastic collision (with coefficient of restitution)
%         %vel_ball_final = (vel_feet*leg_mass*(1+e) + vel_ball*(ball_mass-e*leg_mass)) / (leg_mass+ball_mass);
% 
%         % Resultant Force
%         force = ball_mass*(vel_ball_final-vel_ball)/t;      %f = m(v-u)/t
%     else
% 
%     %if pos_z == 0
%     decel_friction = mu*g*direction;                                        %ff = uN
%     decel_drag = 0.5*rho*norm(vel_ball)^2*C_d*A/ball_mass*direction;        %a = pV^2C_dA/2m
% 
%     decel_tot = decel_drag + decel_friction;
% 
%     vel_ball_final = vel_ball - decel_tot*t;                %v = u +at
%     force = ball_mass*(vel_ball_final-vel_ball)/t;          %f = m(v-u)/t

%% Test
    

    direction = [1 1];
    
    f_friction = mu*ball_mass*g*direction;                              %ff = uN
    f_drag = 0.5*rho*norm(vel_ball)^2*C_d*A*direction;        %a = pV^2C_dA/2m
    
% 
    force_out = force - f_friction - f_drag;

    v_dot = force_out/ball_mass;

    vel_out = vel_ball + v_dot*t;
%     vel_out = 0.98*vel_ball;


end
      

% Goal scored
%if ((ball_position(1,1)<=0) && (ball_position(2,1)>41) && (ball_position(2,1)<59))
%    goal=1; 
%end
%
%if ((ball_position(1,1)>=200) && (ball_position(2,1)>41) && (ball_position(2,1)<59))
%    goal=1; 
%end