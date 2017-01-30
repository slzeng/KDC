clear all
clc



% x0 = [0 0 0;
% 	  0 0 0;
% 	  0 0 0;
% 	  0 0 0];
x0 = [    2.2928  -13.5518   -1.3551;
    0.9862   -9.7395   -2.5839;
   -0.0990   -6.8811   1.2320;
   -1.4951   -2.7670   -0.2013];
goal_pose = [0.1; -.15; 2; 1; 0; 0; 0];
link_length = [1;1;1;1];
obstacles = [0.1 0 1.5 .5;
			 -0.15 .08 .25 .25];

% x0 = [0 0 0;
% 	  0 0 0;
% 	  0 0 0;
% 	  0 0 0];
% goal_pose = [0.1; -.15; 2; 1; 0; 0; 0];
% link_length = [1;1;1;1];
% obstacles = [0.1 0 1.5 .5];



fmin_fun = @(x) obj_fun(x,goal_pose,link_length);
constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);

tic
x = fmincon(fmin_fun,x0,[],[],[],[],[],[],constraint_fun)
toc


visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(1,4)*X+obstacles(1,1),obstacles(1,4)*Y+obstacles(1,2),obstacles(1,4)*Z+obstacles(1,3))
surf(obstacles(2,4)*X+obstacles(2,1),obstacles(2,4)*Y+obstacles(2,2),obstacles(2,4)*Z+obstacles(2,3))


r = x(:,1);
p = x(:,2);
y = x(:,3);
[state,points] = forward_kinimatics(r,p,y,link_length);
state
