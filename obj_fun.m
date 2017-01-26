


%% obj_fun: function description
function [d] = obj_fun(x,goal_pose,link_length)
	r = x(:,1);
	p = x(:,2);
	y = x(:,3);
	
	[pose] = forward_kinimatics(r,p,y,link_length);
	d = sum((pose-goal_pose).^2);
end

