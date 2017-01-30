clear all
clc



x0 = [0 0 0;
	  0 0 0;
	  0 0 0;
	  0 0 0];
goal_pose = [0.1; -.15; 2; 1; 0; 0; 0];
link_length = [1;1;1;1];
obstacles = [0.1 0 1.5 .5];


tic
J = getJacobian(link_length); 
toc

fmin_fun = @(x) obj_fun2(x,goal_pose,link_length,J);
% fmin_fun = @(x) obj_fun(x,goal_pose,link_length);

constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);
options = optimoptions('fmincon','SpecifyObjectiveGradient',true,'Algorithm','interior-point');

tic
x = fmincon(fmin_fun,x0,[],[],[],[],[],[],constraint_fun,options)
toc


visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(4)*X+obstacles(1),obstacles(4)*Y+obstacles(2),obstacles(4)*Z+obstacles(3))

r = x(:,1);
p = x(:,2);
y = x(:,3);
[state,points] = forward_kinimatics(r,p,y,link_length);
state
