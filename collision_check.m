
%% collision_check: function description
function [collision] = collision_check(x,link_length,obstacles)
	r = x(:,1);
	p = x(:,2);
	y = x(:,3);
	[pose,joint_poses] = forward_kinimatics(r,p,y,link_length);
	for(i = 1:size(obstacles,1))
		for(j = 1:(size(joint_poses,1)-1))
			does_intersect = sphere_line_intersection(joint_poses(j,1:3),joint_poses(j+1,1:3),obstacles(i,:));
			if(does_intersect)
				collision = 1;
				return;
			end
		end
	end
	collision = 0;
end

%% sphere_line_intersection: function description
function [does_intersect] = sphere_line_intersection(p1,p2,s)
	
	v = [p2(1) - p1(1); p2(2) - p1(2); p2(3) - p1(3)];
	a = v(1);
	b = v(2);
	c = v(3);
	px = p1(1);
	py = p1(2);
	pz = p1(3);
	x0 = s(1);
	y0 = s(2);
	z0 = s(3);
	r = s(4);

	fun = @(a,b,c,px,py,pz,r,x0,y0,z0)[(-a.*px-b.*py-c.*pz+a.*x0+b.*y0+c.*z0+sqrt(-a.^2.*py.^2-a.^2.*pz.^2-b.^2.*px.^2-b.^2.*pz.^2+a.^2.*r.^2-c.^2.*px.^2-c.^2.*py.^2+b.^2.*r.^2+c.^2.*r.^2-a.^2.*y0.^2-b.^2.*x0.^2-a.^2.*z0.^2-c.^2.*x0.^2-b.^2.*z0.^2-c.^2.*y0.^2+b.^2.*px.*x0.*2.0+a.^2.*py.*y0.*2.0+c.^2.*px.*x0.*2.0+a.^2.*pz.*z0.*2.0+c.^2.*py.*y0.*2.0+b.^2.*pz.*z0.*2.0+a.*b.*px.*py.*2.0+a.*c.*px.*pz.*2.0+b.*c.*py.*pz.*2.0-a.*b.*py.*x0.*2.0-a.*b.*px.*y0.*2.0-a.*c.*pz.*x0.*2.0-a.*c.*px.*z0.*2.0-b.*c.*pz.*y0.*2.0-b.*c.*py.*z0.*2.0+a.*b.*x0.*y0.*2.0+a.*c.*x0.*z0.*2.0+b.*c.*y0.*z0.*2.0))./(a.^2+b.^2+c.^2);-(a.*px+b.*py+c.*pz-a.*x0-b.*y0-c.*z0+sqrt(-a.^2.*py.^2-a.^2.*pz.^2-b.^2.*px.^2-b.^2.*pz.^2+a.^2.*r.^2-c.^2.*px.^2-c.^2.*py.^2+b.^2.*r.^2+c.^2.*r.^2-a.^2.*y0.^2-b.^2.*x0.^2-a.^2.*z0.^2-c.^2.*x0.^2-b.^2.*z0.^2-c.^2.*y0.^2+b.^2.*px.*x0.*2.0+a.^2.*py.*y0.*2.0+c.^2.*px.*x0.*2.0+a.^2.*pz.*z0.*2.0+c.^2.*py.*y0.*2.0+b.^2.*pz.*z0.*2.0+a.*b.*px.*py.*2.0+a.*c.*px.*pz.*2.0+b.*c.*py.*pz.*2.0-a.*b.*py.*x0.*2.0-a.*b.*px.*y0.*2.0-a.*c.*pz.*x0.*2.0-a.*c.*px.*z0.*2.0-b.*c.*pz.*y0.*2.0-b.*c.*py.*z0.*2.0+a.*b.*x0.*y0.*2.0+a.*c.*x0.*z0.*2.0+b.*c.*y0.*z0.*2.0))./(a.^2+b.^2+c.^2)];	

	t = fun(a,b,c,px,py,pz,r,x0,y0,z0);

	if(all(~isreal(t)))
		does_intersect = false;
		return;
	end
	if(any(t<1))
		does_intersect = true;
	else
		does_intersect = false;
	end

end




