% Fits a wall to a given set of points using least squares.
%
% Input:        points              2xt
%
% Output:       wall                5x1     Wall coefficients in Hessian normal form [a,b,c, s,e]

function wall = fit_wall(points)
	pointst = [points(2,:)
	           points(1,:)];
	
	[wall,  e] = fit_it(points);
	[wallt, et]= fit_it(pointst);
	
	if et < e
		wall = [wallt(2)
		        wallt(1)
				wallt(3)];
	end
	
	proj_lengths = wall_orthogonal(wall)'*points;
	
	wall = [wall
			min(proj_lengths)
			max(proj_lengths)];
end

function [wall, e] = fit_it(points)
	[bc, S] = polyfit(points(1,:), points(2,:), 1);
	
	e = S.normr;
	
	orth = [1 bc(1)];
	orth = orth/norm(orth);
			
	wall = [-orth(2); %turn orth 90 degrees counter clockwise to get normal
			orth(1);
			bc(2)*orth(1)];
end

function distances = wall_distance(wall, points)
	distances = wall(1:2)' * points;
end

function orth = wall_orthogonal(wall)
	orth = [ wall(2)
			-wall(1)];
end
