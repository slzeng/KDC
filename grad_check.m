clear all
clc

link_length = [1;1];
tic
J = getJacobian(link_length); 
toc


x0 = [3.1415/4 -3.1415/4 3.1415/2;
	  3.1415/2 3.1415/2 -1];

%; 1; 0; 0; 0
goal_pose = [0; 2; 0];
obstacles = [0.1 0 1.5 .5];


dx = .0001;
x_offset = [0 0 0;
	  		0 dx 0];


fmin_fun = @(x) obj_fun2(x,goal_pose,link_length,J);

[s0,ds0] = fmin_fun(x0)
[s,ds] = fmin_fun(x0+x_offset)

(s-s0)/dx

visualize_arm(x0,link_length);