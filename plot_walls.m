% Plots walls on 
%
% Input:        walls               5xT         Wall coefficients in Hessian normal form [a,b,c, s,e]



function plot_walls(walls, varargin)
	restore_hold_off = ~ishold;
	hold on;

	for i=1:size(walls,2)
		wall = walls(:,i);
		normal = wall(1:2);
		offset = normal*wall(3);
		orth = [normal(2);-normal(1)];
		
		s = orth*wall(4)+offset;
		e = orth*wall(5)+offset;
		plot([s(1) e(1)], [s(2) e(2)], varargin{1:end});
	end
	
	if restore_hold_off
		hold off;
	end
end
