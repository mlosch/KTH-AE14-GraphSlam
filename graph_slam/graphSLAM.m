%GRAPHSLAM
%
% Inputs:       poses               3xt
%               delta_poses         3xt
%               measurements        6xt 
%                                   (e.g. 6 range measurements per timestep)
%               correspondences     6xt     correspondence to wall per measurement
%               R                   3x3     Process noise
%               Q                   1x1     Measurement noise
%               TF                  Transform
%               maxIterations       1x1
%
% Output:       corrected_poses     3xt
function poses = graphSLAM( poses, delta_poses, measurements, correspondences, R, Q, TF, maxIterations )

    x = @(i)     3*i-2 :     3*i;

    T = size(poses,2);

    for i=1:maxIterations
        
        walls = fit_walls(poses, measurements, correspondences, TF);
        pairs = pair_plausible_walls(correspondences,walls)
        
        [omega, xi, tau] = linearize(poses, delta_poses, measurements, correspondences, walls, R, Q, TF);
        %[omega_reduced, xi_reduced] = reduce(omega, xi, tau);
        %[corrected_poses, ~] = solve(omega_reduced, xi_reduced, omega, xi, tau);
        

        fprintf('Iteration %d: Solving...\n',i);
        fprintf('NaN count: %d\n', sum(find(omega~=omega)));
        
%Attempt 1:
%         omega(1:3,1:3) = eye(3);
%         omega = (omega + omega')/2;
%         HS=sparse(omega);
%         deltax = HS\xi;
%         %deltax = conjgrad(omega,xi);
%         
%         poses = poses + reshape(deltax(1:3*T), 3, T);
       
%Attempt 2:
%         error = 0;
%         for t=1:T
%             ind = x(t);
%             poses(:,t) = poses(:,t) + omega(ind,ind) \ xi(ind,1);
%             
%             error = error + sum(omega(ind,ind) \ xi(ind,1));
%         end
%         poses(3,:) = wrapToPi(poses(3,:));
        
% %Attempt 3:
        omega = omega(1:T*3,1:T*3);
        xi = xi(1:T*3,1);
        
        HS = sparse(omega);
        deltax = HS\xi;%conjgrad((HS+HS')/2, xi);
        
        poses = poses + reshape(deltax(1:3*T), 3, T);
        poses(3,:) = wrapToPi(poses(3,:));

%Error calculation:
%        error = sum(sum(abs(deltax)));
        %fprintf('Error: %f\n',error);
        
%        if error < 1e-6
%            break;
%        end

        clf;
        plot_map(poses,measurements,correspondences,TF)
        axis equal
        pause(0.1);
        
    end
    
end

