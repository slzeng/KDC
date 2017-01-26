clear all
clc



x0 = [0 0 0;
	  0 0 0;
	  0 0 0];
goal_pose = [-0.1; 1; 1.5];
link_length = [1;1;1];
obstacles = [0.1 0 1.5 .5];
fmin_fun = @(x) obj_fun(x,goal_pose,link_length);
constraint_fun = @(x) deal((collision_check(x,link_length,obstacles)),[]);


x = fmincon(fmin_fun,x0,[],[],[],[],[],[],constraint_fun)



visualize_arm(x,link_length);
[X,Y,Z] = sphere;
surf(obstacles(4)*X+obstacles(1),obstacles(4)*Y+obstacles(2),obstacles(4)*Z+obstacles(3))
