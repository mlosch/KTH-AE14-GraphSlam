function means = solve( omega_reduced, xi_reduced, omega, xi, tau )
%solve
%
% Inputs:   omega_reduced
%           xi_reduced
%           omega
%           xi
%           tau             Txm
%
% Outputs:  means           Corrected poses

T = size(xi_reduces,1);
%covariances = zeros(3,3,T);
means = zeros(3,T);

%assumes dimension of 2 per feature
nFeatures = (size(omega,1) - size(omega_reduced,1))/2;

for t=1:T
    [idxY, idxX] = o(t,0,3,3);
    means(:,t) = omega_reduced(idxY,idxX) \ xi_reduced(idxY,1);
end

pose_indices = 1:T;

for j=1:nFeatures
    tau_j = pose_indices(tau(:,j));
    
    for t=1:length(tau_j)
        
        [indY_xi, ~] = o(T,j,2,2);
        [indY_om, ~] = o(T,j,3,3);
        [~, indX_om] = o(t,0,3,3);
        
        %line 6:
        means(:,t) = omega(indY_om, indY_om) \ (xi_j(indY_xi,1) + omega(indY_om, indX_om)*means(:,t));
    end
end

end

