
%% visualize_arm: function description
function visualize_arm(x,link_length)
	r = x(:,1);
	p = x(:,2);
	y = x(:,3);
	[state,points] = forward_kinimatics(r,p,y,link_length);
	figure(1)
	clf
	hold on
	xlabel('x')
	ylabel('y')
	zlabel('z')
	axis equal
	plot3(0,0,0,'rx')
	% x = [0; 0; 0;1];
	% points = x';
	% % R = eye(3);
	% H = eye(4);
	% for(i = 1:length(r))
	% 	R = eul2rotm([y(i),p(i),r(i)]);
	% 	% R = R';
	% 	% Rx = [1 0 0; 0 cos(r(i)) -sin(r(i)); 0 sin(r(i)) cos(r(i))];
	% 	% Ry = [cos(p(i)) 0 sin(p(i)); 0 1 0; -sin(p(i)) 0 cos(p(i))];
	% 	% Rz = [cos(y(i)) -sin(y(i)) 0; sin(y(i)) cos(y(i)) 0; 0 0 1];
	% 	% R = (Rx*Ry*Rz);
	% 	% R = R';
	% 	% x = x + R*[link_length(i);0;0];
	% 	H = H*([R, R*[link_length(i);0;0]; 0 0 0 1]);
	% 	% H = H_new*H;
	% 	x = H*[0;0;0;1]
	% 	points = [points;x'];
	% end

	plot3(points(:,1),points(:,2),points(:,3),'bo-')
end