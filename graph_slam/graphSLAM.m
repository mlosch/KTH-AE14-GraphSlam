%GRAPHSLAM
%
% Inputs:       poses               3xt
%               measurements        6xt 
%                                   (e.g. 6 range measurements per timestep)
%               correspondences     6xt     correspondence to wall per measurement
%               R                   3x3     Process noise
%               Q                   1x1     Measurement noise
%               TF                  Transform
%               maxIterations       1x1
%
% Output:       corrected_poses     3xt
function corrected_poses = graphSLAM( poses, measurements, correspondences, R, Q, TF, maxIterations )

    for i=1:maxIterations
        
        walls = fit_walls(poses, irs, correspondences, TF);
        
        [omega, xi, tau] = linearize(poses, measurements, correspondences, walls, R, Q, TF);
        [omega_reduced, xi_reduced] = reduce(omega, xi, tau);
        [corrected_poses, ~] = solve(omega_reduced, xi_reduced, omega, xi, tau);
        
        err = sum(sum(abs(corrected_poses-poses)));
        
        if err < 1e-6
            break;
        end
        
    end

end

