% Plots all the walls
%
% Input:        walls               4xT         Wall coefficients in something like Hessian normal form [Nu,c, s,e]
%               ...                             Arguments to pass on to the plot call for each line

function plot_walls(walls, varargin)
	restore_hold_off = ~ishold;
	hold on;

	for i=1:size(walls,2)
		wall = walls(:,i);
		normal = [cos(wall(1))
		          sin(wall(1))];
		offset = normal*wall(2);
		orth = [normal(2);-normal(1)];
		
		s = orth*wall(3)+offset;
		e = orth*wall(4)+offset;
		plot([s(1) e(1)], [s(2) e(2)], varargin{1:end});
	end
	
	if restore_hold_off
		hold off;
	end
end
