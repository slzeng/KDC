



%% forward_kinimatics: function description
function [x,points] = forward_kinimatics(r,p,y,link_length)
	
	x = [0; 0; 0;1];
	points = x';
	% R = eye(3);
	H = eye(4);
	for(i = 1:length(r))
		R = eul2rotm([y(i),p(i),r(i)]);
		% R = R';
		% Rx = [1 0 0; 0 cos(r(i)) -sin(r(i)); 0 sin(r(i)) cos(r(i))];
		% Ry = [cos(p(i)) 0 sin(p(i)); 0 1 0; -sin(p(i)) 0 cos(p(i))];
		% Rz = [cos(y(i)) -sin(y(i)) 0; sin(y(i)) cos(y(i)) 0; 0 0 1];
		% R = (Rx*Ry*Rz);
		% R = R';
		% x = x + R*[link_length(i);0;0];

		H = H*([R, R*[link_length(i);0;0]; 0 0 0 1]);
		% H = H_new*H;
		x = H*[0;0;0;1];
		points = [points;x'];
	end
	rotm = H(1:3,1:3);
	x = x(1:3);
	q = rotm2quat(rotm);
	x = [x;q'];
end