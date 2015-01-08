% Fits all the walls given by the poses, their ir measurements, and the
% ir measurement's assigned wall IDs (correspondences)
%
% Input:        poses               tx3
%               ir                  tx6
%               correspondences     tx6         The ir measurements assigned wall IDs
%               tf                  Transform   For transforming ir distances to world space



function plot_walls(poses, ir, correspondences, tf)
	t = size(poses, 1);
	clf
	hold on
	
	%walls
	walls = fit_walls(poses, ir, correspondences, tf);
	for i=1:size(walls,1)
		wall=walls(i,:);
		normal = wall(1:2);
		offset = normal*wall(3);
		orth = [normal(2) -normal(1)];
		
		s = orth*wall(4)+offset;
		e = orth*wall(5)+offset;
		plot([s(1) e(1)], [s(2) e(2)]);
	end
	
	%ir
	ir_points = tf.transform_to_map_multiple(poses, ir);
	plot(ir_points(1,:), ir_points(2,:), '.g');
	
	%poses
	plot(poses(:,1), poses(:,2), 'or');
end
