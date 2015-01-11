function [ omega, xi, tau ] = linearize( poses, delta_poses, measurements, correspondences, walls, R, Q, TF )
%LINEARIZE
% Inputs:       poses               3xt
%               delta_poses         3xt
%               measurements        6xt 
%                                   (e.g. 6 range measurements per timestep)
%               correspondences     6xt     correspondence to wall per measurement
%               walls               4xn     of type [Nu, c, start, end]
%               R                   3x3     Process noise
%               Q                   1x1     Measurement noise
%               TF                  Transform
%
% Output:       omega               nxm     information matrix
%               xi                  nxm     information vector
%               tau                 Txm     Binary matrix that maps
%               observed features to poses

%assert(isequal(poses(:,1), [0;0;0]),'Assertion failed at linearize(): First pose has to be [0 0 0]');

T = size(poses,2);

x = @(i)     3*i-2 :     3*i;
xc= @(i)     3*i-2 :     3*i+3;
m = @(i) 3*T+2*i-1 : 3*T+2*i;

nWalls = max(max(correspondences));

omega = zeros(3*T+2*nWalls);
xi = zeros(3*T+2*nWalls,1);
tau = false(T,nWalls);

omega(1:3,1:3) = diag([inf inf inf]);

R_inv = inv(R);

%dynamics
for t=2:T
    
    mu_t_1 = poses(:,t-1);
    x_hat_t = mu_t_1 + delta_poses(:,t);
    
    theta_t_1 = mu_t_1(3);
    
    ds = norm(delta_poses(1:2,t));
    %ds = norm(poses(1:2,t-1)-poses(1:2,t));
    
%     v_t = norm(delta_poses(1:2,t))/0.1;
%     w_t = delta_poses(3,t)/0.1;
%     w_t = w_t + 1e-15;

% Method from line 5:
%     x_hat_t = mu_t_1;
%     x_hat_t(1) = -v_t/w_t * sin(theta_t_1) + v_t/w_t * sin(theta_t_1 + w_t*0.1);
%     x_hat_t(2) = v_t/w_t * cos(theta_t_1) - v_t/w_t * cos(theta_t_1 + w_t*0.1);
%     x_hat_t(3) = w_t*0.1;
    
    G_t = eye(3);
    G_t(1,3) = -delta_poses(2,t)/0.1;%-ds * sin(theta_t_1);
    G_t(2,3) = delta_poses(1,t)/0.1;%ds * cos(theta_t_1);
 
% Method from line 6:
%    G_t(1,3) = v_t/w_t * cos(theta_t_1) - v_t/w_t * cos(theta_t_1 + w_t*0.1);
%    G_t(2,3) = v_t/w_t * sin(theta_t_1) - v_t/w_t * sin(theta_t_1 + w_t*0.1);
    
    M = [eye(3); -G_t] * R_inv * [eye(3), -G_t];
    %M = [-G_t; eye(3)] * R_inv * [-G_t, eye(3)];
    
%     omega(xt1,xt1) = omega(xt1,xt1) + M(1:3,1:3);
%     omega(xt1,xt ) = omega(xt1,xt ) + M(4:6,1:3);
%     omega(xt ,xt1) = omega(xt ,xt1) + M(1:3,4:6);
%     omega(xt ,xt ) = omega(xt ,xt ) + M(4:6,4:6);
    
    ind = xc(t-1);
    omega(ind,ind) = omega(ind,ind) + M;
    
    K = [eye(3); -G_t] * R_inv * [x_hat_t + G_t*mu_t_1];
    %K = [-G_t; eye(3)] * R_inv * [x_hat_t + G_t*mu_t_1];
    xi(ind,1) = xi(ind,1) + K;
    
end

%skip measurement innovations for now
return;

Q_inv = 1/Q;
%measurements
for t=2:T
    
    poseTF = TF.pose_to_transform_mat(poses(:,t));
    
    %for all observed features
    for i=1:6
        
        j = correspondences(i,t);
        
        if (j <= 0) continue; end;
        
        %get wall feature j
        wall = walls(:,j);
        
        %add flag to tau
        tau(t,j) = true;
        
        %sensor offsets and angles
        ir_offsets = TF.ir_offsets(:,i);
        ir_angle = TF.ir_angles(1,i);
        
        ir_dist = measurements(i,t);
        
        z_hat = observation_model(poses(:,t), wall, measurements(:,t), i, TF);
        
        %observation_model_derivative(pose,wall,sensor_offsets,sensor_alignment)
        H = observation_model_derivative(poses(:,t), wall, ir_offsets, ir_angle);
        
        H_tp = H' * Q_inv;
        
        H_tp_omega = H_tp * H; %line 18
        
        %decompose H_tp_omega
        indY = x(t);
        indX = indY;
        omega(indY,indX) = omega(indY,indX) + H_tp_omega(1:3,1:3);
        
        indY = m(j);
        omega(indY,indX) = omega(indY,indX) + H_tp_omega(4:5,1:3);
        
        indY = x(t);
        indX = m(j);
        omega(indY,indX) = omega(indY,indX) + H_tp_omega(1:3,4:5);
        
        indY = m(j);
        indX = indY;
        omega(indY,indX) = omega(indY,indX) + H_tp_omega(4:5,4:5);
        
        
        H_tp_xi = H_tp * (ir_dist - z_hat - H * [poses(:,t); wall(1:2,1)]); %line 19
        indY = x(t);
        xi(indY,1) = xi(indY,1) + H_tp_xi(1:3,1);
        
        indY = m(j);
        xi(indY,1) = xi(indY,1) + H_tp_xi(4:5,1);
    end
    
end

end
