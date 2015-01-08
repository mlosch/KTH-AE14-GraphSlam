% Fits all the walls given by the poses, their ir measurements, and the
% ir measurement's assigned wall IDs (correspondences)
%
% Input:        poses               3xt
%               ir                  6xt
%               correspondences     6xt         The ir measurements assigned wall IDs
%               tf                  Transform   For transforming ir distances to world space
%
% Output:       walls               tx5         Wall coefficients in Hessian normal form [a,b,c, s,e]
%                                               t is given by the maximum wall ID in correspondences



function walls = fit_walls(poses, ir, correspondances, tf)
	wallc = max(max(correspondances));
	walls = zeros(wallc, 5);
end
