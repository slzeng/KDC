function [r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
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


x0 = [0 0 0;
	  0 0 0;
	  0 0 0;
	  0 0 0];


lb = [min_roll, min_pitch, min_yaw]; 
ub = [max_roll, max_pitch, max_yaw]; 

tic
J = getJacobian(link_length); 
toc

fmin_fun = @(x) obj_fun2(x,target,link_length,J);
% fmin_fun = @(x) obj_fun(x,goal_pose,link_length);

constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);
options = optimoptions('fmincon','SpecifyObjectiveGradient',true,'Algorithm','interior-point');

tic
x = fmincon(fmin_fun,x0,[],[],[],[],lb,ub,constraint_fun,options)
toc


visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(4)*X+obstacles(1),obstacles(4)*Y+obstacles(2),obstacles(4)*Z+obstacles(3))

r = x(:,1);
p = x(:,2);
y = x(:,3);
[state,points] = forward_kinimatics(r,p,y,link_length);
state

end
