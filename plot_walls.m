% Fits all the walls given by the poses, their ir measurements, and the
% ir measurement's assigned wall IDs (correspondences)
%
% Input:        poses               tx3
%               ir                  tx6
%               correspondences     6xt         The ir measurements assigned wall IDs
%               tf                  Transform   For transforming ir distances to world space



function plot_walls(poses, ir, correspondences, tf)
	t = size(poses, 1);
	
	if ~isequal(size(correspondences), [6 t])
		error('is correspondances-matrix not transposed correctly?');
	end
	
	clf
	hold on

	ir_points = tf.transform_to_map_multiple(poses, ir);
	walls = fit_walls(poses, ir, correspondences, tf);
	
	%ir
	for i=1:size(walls,2)
		wall=walls(:,i);
		corr_ir_points = ir_points(:,correspondences == i);
		
		proj = corr_ir_points - wall(1:2) * (wall(1:2)'*corr_ir_points - wall(3));
		for j=1:size(corr_ir_points,2)
			plot([corr_ir_points(1,j) proj(1,j)], [corr_ir_points(2,j) proj(2,j)], '-g');
		end
	end
	plot(ir_points(1,:), ir_points(2,:), '.g');
	
	%walls
	for i=1:size(walls,2)
		wall=walls(:,i);
		normal = wall(1:2);
		offset = normal*wall(3);
		orth = [normal(2);-normal(1)];
		
		s = orth*wall(4)+offset;
		e = orth*wall(5)+offset;
		plot([s(1) e(1)], [s(2) e(2)]);
	end
	
	%poses
	plot(poses(:,1), poses(:,2), 'or');
	
end
