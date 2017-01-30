


%% obj_fun: function description
function [score, derivative] = obj_fun2(x,goal_pose,link_length,J)
	rA = x(:,1);
	pA = x(:,2);
	yA = x(:,3);
	
	[pose] = forward_kinimatics(rA,pA,yA,link_length);
	score = sum((pose-goal_pose).^2);
    
    %sub values into the Jacobian 
    % roll = sym('r',[1, length(link_length)]);
    % pitch = sym('p', [1, length(link_length)]);
    % yaw = sym('y', [1, length(link_length)]);
    % % J = subs(J,{roll, pitch, yaw},{rA, pA, yA});
    % for i = 1:length(rA) 
    %     J = subs(J,{roll(i), pitch(i), yaw(i)},{rA(i), pA(i), yA(i)});
    % end 



    %compile the derivative 
    % [rA',pA',yA']
    % 'J'
    % J([rA',pA',yA'])
    derivative = double(2*(pose - goal_pose)'*J([rA',pA',yA']))';
    derivative = reshape(derivative,[3,length(link_length)])';
    % derivative = ones(3,length(link_length));
    % derivative = zeros(length(link_length)*3,1); 
    % for j = 1:length(link_length)*3
    %     derivative(j) = 2*(pose(1) - goal_pose(1))*J(1,j) + ...
    %                     2*(pose(2) - goal_pose(2))*J(2,j) + ...
    %                     2*(pose(3) - goal_pose(3))*J(3,j);
    % end 
end

