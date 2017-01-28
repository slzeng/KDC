clear all
clc



x0 = [0 0 0;
	  0 0 0;
	  0 0 0];
goal_pose = [1; 2; 0];
link_length = [1;1;1];
obstacles = [0.1 0 1.5 .5];

J = getJacobian(link_length); 

fmin_fun = @(x) obj_fun2(x,goal_pose,link_length,J);
constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);
options = optimoptions('fmincon','SpecifyObjectiveGradient',true);
x = fmincon(fmin_fun,x0,[],[],[],[],[],[],constraint_fun,options)



visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(4)*X+obstacles(1),obstacles(4)*Y+obstacles(2),obstacles(4)*Z+obstacles(3))
