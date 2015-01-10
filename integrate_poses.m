% Integrates poses
%
% Input:        delta_poses         3xt
%
% Output:       poses               3xt     integrated poses

function poses = integrate_poses(delta_poses)
	assert(size(delta_poses,1) == 3, 'is delta_poses-matrix transposed correctly?');
	
	delta_poses(:,1) = 0; %force first pose to be (0,0,0)
	
	dx = delta_poses(1,:);
	dy = delta_poses(2,:);
	dt = delta_poses(3,:);
	theta = wrapToPi(cumsum(dt));
	c = cos(theta);
	s = sin(theta);
	x = cumsum(dx.*c-dy.*s);
	y = cumsum(dx.*s+dy.*c);
	
	poses = [x
	         y
			 theta];
end
