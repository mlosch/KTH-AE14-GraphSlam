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
function poses = graphSLAM( poses, measurements, correspondences, R, Q, TF, maxIterations )

    T = size(poses,2);

    for i=1:maxIterations
        
        walls = fit_walls(poses, measurements, correspondences, TF);
        
        [omega, xi, tau] = linearize(poses, measurements, correspondences, walls, R, Q, TF);
        %[omega_reduced, xi_reduced] = reduce(omega, xi, tau);
        %[corrected_poses, ~] = solve(omega_reduced, xi_reduced, omega, xi, tau);
        

        fprintf('Iteration %d: Solving...\n',i);
        %omega(1:3,1:3) = omega(1:3,1:3) + eye(3);
        omega = (omega + omega')/2;
        
        fprintf('NaN count: %d\n', sum(find(omega~=omega)));
        
        HS=sparse(omega);
        deltax = HS\xi;
        
        poses = poses + reshape(deltax(1:3*T), 3, T);
        
        change = sum(sum(abs(deltax)));
        fprintf('Improvement: %f \n', change);
        
        if change < 1e-6
            break;
        end
        
    end
    
end

