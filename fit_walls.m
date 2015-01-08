% Fits all the walls given by the poses, their ir measurements, and the
% ir measurement's assigned wall IDs (correspondences)
%
% Input:        poses               tx3
%               ir                  tx6
%               correspondences     tx6         The ir measurements assigned wall IDs
%               tf                  Transform   For transforming ir distances to world space
%
% Output:       walls               5xs         Wall coefficients in Hessian normal form [a,b,c, s,e]
%                                               s is given by the maximum wall ID in correspondences



function walls = fit_walls(poses, ir, correspondences, tf)
	t = size(poses, 1);
	
	wallc = max(max(correspondences));
	walls = zeros(wallc, 5);
	
	ir_points = tf.transform_to_map_multiple(poses, ir);
	for i = 1:wallc
		corresponding_points = ir_points(:, correspondences == i);
		
		if isempty(corresponding_points)
			error('no assigned points for class');
		end
		walls(i,:) = fit_wall(corresponding_points);
	end
end
