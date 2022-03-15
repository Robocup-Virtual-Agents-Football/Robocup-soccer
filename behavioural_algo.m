function def = defender_role(obj, field)
if obj.behaviourRole == player.DEFENDER
    if checkPossession(obj) == true 
        if checkInRangeS(obj) == true   %make assumption no need to write, remove - add this report as it's assumption
            if checkTeamMatefree(obj) == true
                obj = behavior_kick(obj,field); 
            else
                avoidCollision(obj);
            end
        else
            obj = behavior_move(obj,cfg.field_half);
            if obj.Pos > cfg.field_half
                roleSwitching(obj, field)  %define
            else
                checkInRange(obj)
            end
        end
    else
         if checkInRangeL(obj) == true     % 2 check in range functions?
            if checkTeamMatefree(obj) == true   %check for this
                obj = behavior_kick(obj,field);   %pass the ball to your teammte
            else
                avoidCollision(obj);
            end
        else
            obj = behavior_move(obj,cfg.field_half);
            if obj.Pos > cfg.field_half
                roleSwitching(obj, field)
            else
                checkInRange(obj)
            end
         end
     
function attack = attacker_role(obj, field)
if obj.behaviourRole == player.ATTACKER
    if checkPossession(obj) == true
        if checkInRangeS(obj) == true
            if checkTeamMatefree(obj) == true
                obj = behavior_kick(obj,field); 
            else
                avoidCollision(obj);
            end
        else
            obj = behavior_move(obj,cfg.field_half);
            if obj.Pos > cfg.field_half
                roleSwitching(obj, field)
            else
                checkInRange(obj)
            end
        end
    else
         if checkInRangeL(obj) == true
            if checkTeamMatefree(obj) == true
                obj = behavior_kick(obj,field); 
            else
                avoidCollision(obj);
            end
        else
            obj = behavior_move(obj,cfg.field_half);
            if obj.Pos > cfg.field_half
                roleSwitching(obj, field)
            else
                checkInRange(obj)
            end
         end
                
       

        



function obj = behavioural_algo(obj,field) 
%BEHAVIOR runs simple fsm to simulate player behavior

%simple FSM
if obj.behaviorState == player.KICK
    obj = behavior_kick(obj,field);        
elseif obj.behaviorState == player.MOVE
    obj = behavior_move(obj,field);
elseif obj.behaviorState == player.SEARCH;
    obj = behavior_search(obj,field);
elseif obj.behaviorState == player.APPROACH;
    obj = behavior_approach(obj,field);
else
    obj.behaviorState = player.SEARCH;
    obj = behavior_search(obj,field);
end

end


function obj = behavior_move(obj,field)

%update previous ball info if we don't have current info
if isempty(field.cur_player.ball_local)
    
    %check if the teamball is known
    if ~isempty(field.teamball)
        ball_global = field.teamball.pos;
    else
        ball_global = obj.prev_ball.pos;
        
        %do search if ball is lost for too long
        if (obj.gametime - obj.prev_ball.time) > obj.cfg.ballLostTime
            obj.behaviorState = player.SEARCH;
            obj.bh_init = true;    
        end
    end
else
    %update prev_ball info if we currently see it
    ball_global = field.cur_player.pos(1:2) + field.cur_player.ball_local;
    obj.prev_ball.pos = ball_global;
    obj.prev_ball.time = obj.gametime;
end

%do calculations
[ obj, nearPos, ~ ] = obj.behavior_handle{obj.get_bh_id()}(obj,field,ball_global);

%check to see if we need to transition
if nearPos && (obj.role == player.ATTACKER || (obj.role == player.GOALIE && norm(field.cur_player.ball_local) < obj.cfg.GoalieGoThresh))
    obj.behaviorState = player.APPROACH;
    obj.bh_init = true; 
end

end

function obj = behavior_search(obj,field)

%if we don't see the ball, do some simple searching for it  
if isempty(field.teamball)
    
    if obj.bh_init
        dist = obj.prev_ball.pos - field.cur_player.pos(1:2);
        dist_rel = obj.local2globalTF'*[dist';0];
        ang = atan2(dist_rel(2),dist_rel(1));
        dir = sign(ang);
        obj.bh_init = false;
        obj.vel_des = [0,0,dir*obj.cfg.player_MaxAngVel/2]; 
    end           
            
%if we do see the ball, update prev info and switch to move
else
    obj.prev_ball.pos = field.teamball.pos;
    obj.prev_ball.time = obj.gametime;
    obj.behaviorState = player.MOVE;
    obj.bh_init = true;
end

end

function obj = behavior_kick(obj,field)

obj.kick = 1;
obj.behaviorState = player.SEARCH;
obj.bh_init = true;

end

function obj = behavior_approach(obj,field)

%check to make sure we can still see the ball
if isempty(field.cur_player.ball_local)
    obj.behaviorState = player.SEARCH;
    obj.bh_init = true;
    return
end

if norm(field.cur_player.ball_local) > obj.cfg.closetoPos
    obj.behaviorState = player.MOVE;
    obj.bh_init = true;
end

%get info about the field
ball_global = field.cur_player.pos(1:2) + field.cur_player.ball_local;
pose_global = field.cur_player.pos;
goal_attack = field.goal_attack;

%find desired angle (want to point towards attacking goal)
dpGoal = goal_attack - pose_global(1:2);
ang_des = atan2(dpGoal(2),dpGoal(1));

%find desired position (behind ball in line with goal)
n = [cos(ang_des), sin(ang_des)];
pos_des(1:2) = ball_global - n*(obj.cfg.player_hitbox_radius+obj.cfg.ball_radius);

%stay pointing at ball while moving to position
dpBall = field.cur_player.ball_local;
pos_des(3) = atan2(dpBall(2),dpBall(1));

%update desired pose and calculate velocity
obj.pos_des = pos_des;
[ obj.vel_des,nearPos,nearAng ] = obj.velSimple(field);

% obj.vel_des = 0.1*obj.vel_des;

if nearAng && nearPos
    obj.behaviorState = player.KICK;
    obj.bh_init = true; 
end   


end




