


%% obj_fun: function description
function [score, derivative] = obj_fun2(x,goal_pose,link_length,J)
	rA = x(:,1);
	pA = x(:,2);
	yA = x(:,3);
	
	[pose] = forward_kinimatics(rA,pA,yA,link_length);
	score = sum((pose-goal_pose).^2);
    
    %sub values into the Jacobian 
    roll = sym('r',[1, length(link_length)]);
    pitch = sym('p', [1, length(link_length)]);
    yaw = sym('y', [1, length(link_length)]);
    for i = 1:length(rA) 
        J = subs(J,{roll(i), pitch(i), yaw(i)},{rA(i), pA(i), yA(i)});
    end 

    %compile the derivative 
    derivative = zeros(length(link_length)*3,1); 
    for j = 1:length(link_length)*3
        derivative(j) = 2*(pose(1) - goal_pose(1))*J(1,j) + ...
                        2*(pose(2) - goal_pose(2))*J(2,j) + ...
                        2*(pose(3) - goal_pose(3))*J(3,j);
    end 
end

