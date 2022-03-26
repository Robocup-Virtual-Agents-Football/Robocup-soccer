% function force  = fcn(vel_ball, robot_position, feet_state, strategy, ball_position)
%--------------------|  1x2  |----|  Nx2  |-------| Nx1 |----| Nx1 |----| 1x2 |---
function [out_vel, out_pos] = BallDynamicsTest3(in_vel, in_pos)
 
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
MinBallVel = 0.1;
friction = [2, 2];

if norm(in_vel) ~= 0
    dv = in_vel/norm(in_vel).*friction*delta_t;
    dvTooLarge = abs(dv)>abs(in_vel);
    out_vel = in_vel - dv;
    vTooSmall = abs(out_vel) < MinBallVel;
    
    if any(vTooSmall)
        out_vel(vTooSmall) = 0;
    end
    if any(dvTooLarge)
        out_vel(dvTooLarge) = 0;
    end

    dp = out_vel*delta_t;
    out_pos = in_pos + dp;
else
    out_vel = in_vel;
    out_pos = in_pos;
end            


%% Test
    
end