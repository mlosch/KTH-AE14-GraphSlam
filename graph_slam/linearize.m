function [ omega, xi ] = linearize( poses, measurements, correspondences, R, Q, TF )
%LINEARIZE
% Inputs:       poses               3xt
%               measurements        2x6xt 
%                                   (e.g. 6 measurements [range, bearing] at timestep t)%                                   
%               correspondences     6xt     correspondence per measurement
%               R                   3x3     Process noise
%               Q                   2x2     Measurement noise
%               TF                  Transform
%
% Output:       omega               nxm     information matrix
%               xi                  nxm     information vector

%TODO: Ensure first pose is (0,0,0)

T = size(poses,2);

omega = zeros(4,4,T);
xi = zeros(4,T);

omega(1,:,:) = eye(4)*inf;
omega(1,4,4) = 0;

R_inv = inv(R);

%dynamics
for t=2:T
    
    x_hat_t_1 = poses(:,t-1);
    x_hat_t = poses(:,t);
    
    theta_t_1 = x_hat_t_1(3,1);
    
    ds = norm(x_hat_t_1-x_hat_t);
    
    G_t = eye(3);
    G_t(1,3) = -ds * sin(theta_t_1);
    G_t(2,3) = ds * cos(theta_t_1);
    
    M = [ones(1,4); -G_t] * R_inv * [ones(4,1) -G_t];
    omega(t,:,:) = omega(t,:,:) + M;
    omega(t-1,:,:) = omega(t-1,:,:) + M;
    
end

Q_inv = inv(Q);
%measurements
for t=1:T
    
    poseTF = TF.pose_to_transform_mat(poses(:,t));
    
    %for all observed features
    for i=1:6
        
        j = correspondences(i,t);
        
        if (j == -1) continue; end;
        
        dist    = measurements(1,i,t);
        bearing = measurements(2,i,t);
        
        %determine delta
        point = poseTF * [cos(bearing)*dist; sin(bearing)*dist; 1];
        delta = point(1:2);
        
        z_hat = [dist; atan2(delta(2),delta(1)) - poses(3,t)];
        %TODO: Wrap second comp to -pi,pi
        
        H = [dist*delta(1) -dist*delta(2)   0   -dist*delta(1)  dist*delta(2); ...
            delta(2)        delta(1)        -1  -delta(2)       -delta(1)];
        H = H/(dist*dist);
        
        H_tp = H' * Q_inv;
        
        H_tp_omega = H_tp * H;
        omega(t,:,:) = H_tp_omega;
    end
    
end

end