% Fits a wall to a given set of points using least squares.
%
% Input:        points              2xt
%
% Output:       wall                4x1     Wall coefficients in something like Hessian normal form [Nu,c, s,e]
%                                           where Nu = atan2(b,a) and c = c of the Hessian normal form,
%                                           and s, e are the distance along clockwise orthogonal
%                                           to the normal of the wall to the farthest fitted points

function wall = fit_wall(points)
    warning('off','MATLAB:polyfit:RepeatedPointsOrRescale');

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
	
	wall = [atan2(wall(2), wall(1))
			wall(3)
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

function orth = wall_orthogonal(wall)
	orth = [ wall(2)
			-wall(1)];
end
