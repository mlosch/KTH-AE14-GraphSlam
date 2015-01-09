% Fits a wall to a given set of points using least squares.
%
% Input:        points              2xt
%
% Output:       wall                1x5     Wall coefficients in Hessian normal form [a,b,c, s,e]

function wall = fit_wall(points)
	bc = polyfit(points(1,:), points(2,:), 1);
	
	orth = [1 bc(1)];
	orth = orth/norm(orth);
	
	proj_lengths = orth*points;
	
	wall = [-orth(2); %turn orth 90 degrees counter clockwise to get normal
			orth(1);
			bc(2);
			min(proj_lengths);
			max(proj_lengths)];
end
