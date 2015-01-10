% function h = observation_model(x,M,j)
% This function is the implementation of the h function.
% The bearing should lie in the interval [-pi,pi)
% Inputs:
%           x(t)        3X1
%           M           2XN
%           j           1X1
% Outputs:  
%           h           2X1

% The measurement model for one ir sensor
%
% Input:        pose                3x1
%               wall                5x1
%               ir_distances        6x1         All 6 ir measurements from the pose
%               index               1x1         The index of the ir measurement to observe
%               tf                  Transform   For transforming ir distances to world space

function h = observation_model(pose, wall, ir_distances, index, tf)
	points = tf.irs_to_points(ir_distances);
	p = points(1:2,index);
    N = wall(1);
    theta = pose(3);
	
	h = (dot(p, [cos(N), sin(N)]) - wall(2)) / cos(N - theta);
end
