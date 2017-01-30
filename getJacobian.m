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
		R = (Rz*Ry*Rx);
		H = H*([R, R*[link_length(i);0;0]; 0 0 0 1]);
		
    end
    rotm = H(1:3,1:3);
    % q = rotm2quat(rotm);
    q(1) = sqrt(1+rotm(1,1) + rotm(2,2) + rotm(3,3))/2;
    q(2) = (rotm(3,2) - rotm(2,3))/(4*q(1));
    q(3) = (rotm(1,3) - rotm(3,1))/(4*q(1));
    q(4) = (rotm(2,1) - rotm(1,2))/(4*q(1));

    x = H*[0;0;0;1];
    v = cat(1,roll, pitch, yaw); 
    v = v(:);
    x = x(1:3);
    x = [x;q'];
    J = jacobian(x,v);
    J = matlabFunction(J,'Vars',{[roll,pitch,yaw]});
end 