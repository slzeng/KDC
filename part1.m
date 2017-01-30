function [r, p, y] = part1( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% Function that uses optimization to do inverse kinematics for a snake robot

%%Outputs 
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%%Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the 
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.

% Your code goes here.

% x0 = [0 0 0;
% 	  0 0 0;
% 	  0 0 0;
% 	  0 0 0];
x0 = [    2.2928  -13.5518   -1.3551;
    0.9862   -9.7395   -2.5839;
   -0.0990   -6.8811   1.2320;
   -1.4951   -2.7670   -0.2013];




lb = [min_roll, min_pitch, min_yaw]; 
ub = [max_roll, max_pitch, max_yaw]; 

fmin_fun = @(x) obj_fun(x,target,link_length);
constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);

x = fmincon(fmin_fun,x0,[],[],[],[],lb,ub,constraint_fun)



visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(1,4)*X+obstacles(1,1),obstacles(1,4)*Y+obstacles(1,2),obstacles(1,4)*Z+obstacles(1,3))
surf(obstacles(2,4)*X+obstacles(2,1),obstacles(2,4)*Y+obstacles(2,2),obstacles(2,4)*Z+obstacles(2,3))


r = x(:,1);
p = x(:,2);
y = x(:,3);
[state,points] = forward_kinimatics(r,p,y,link_length);
state

end