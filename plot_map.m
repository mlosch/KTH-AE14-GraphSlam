% Fits all the walls given by the poses, their ir measurements, and the
% ir measurement's assigned wall IDs (correspondences)
%
% Input:        poses               3xt
%               ir                  6xt
%               correspondences     6xt         The ir measurements assigned wall IDs
%               tf                  Transform   For transforming ir distances to world space

function plot_map(poses, ir, correspondences, tf)
	t = size(poses, 2);
	
	if isempty(correspondences)
		correspondences = zeros(6, t);
	end
	
	assert(isequal(size(correspondences), [6 t]), 'is correspondances-matrix not transposed correctly?');
	
	restore_hold_off = ~ishold;
	hold on;

	ir_points = tf.transform_to_map(poses, ir);
	walls = fit_walls(poses, ir, correspondences, tf);
	
	%ir
	for i=1:size(walls,2)
		wall=walls(:,i);
		corr_ir_points = ir_points(:,correspondences == i);
		wall_normal = [cos(wall(1))
		               sin(wall(1))];
		proj = corr_ir_points - wall_normal * (wall_normal'*corr_ir_points - wall(2));
		for j=1:size(corr_ir_points,2)
			plot([corr_ir_points(1,j) proj(1,j)], [corr_ir_points(2,j) proj(2,j)], '-g');
		end
	end
	plot(ir_points(1,:), ir_points(2,:), '.g');
	
	%walls
	plot_walls(walls, '-b');
	
	%poses
	plot(poses(1,:), poses(2,:), 'or');
	
	if restore_hold_off
		hold off;
	end
end
