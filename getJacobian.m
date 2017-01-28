function [J] = getJacobian(link_length)
    roll = sym('r',[1, length(link_length)]);
    pitch = sym('p', [1, length(link_length)]);
    yaw = sym('y', [1, length(link_length)]);
    x = [0; 0; 0;1];
	H = eye(4);
    for(i = 1:length(link_length))
		Rx = [1 0 0; 0 cos(roll(i)) -sin(roll(i)); 0 sin(roll(i)) cos(roll(i))];
		Ry = [cos(pitch(i)) 0 sin(pitch(i)); 0 1 0; -sin(pitch(i)) 0 cos(pitch(i))];
		Rz = [cos(yaw(i)) -sin(yaw(i)) 0; sin(yaw(i)) cos(yaw(i)) 0; 0 0 1];
		R = (Rx*Ry*Rz);
		H = H*([R, R*[link_length(i);0;0]; 0 0 0 1]);
		
    end
    x = H*[0;0;0;1];
    v = cat(1,roll, pitch, yaw); 
    v = v(:); 
    J = jacobian(x,v); 
    

end 