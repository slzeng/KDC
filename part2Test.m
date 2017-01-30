%script to run part 2 
target = [0.1; -.15; 2; 1; 0; 0; 0];
link_length = [1;1;1;1];
obstacles = [0.1 0 1.5 .5;
			 -0.15 .08 .25 .25];
min_roll = []; 
max_roll = []; 
min_pitch = []; 
max_pitch = []; 
min_yaw = []; 
max_yaw = []; 

[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )