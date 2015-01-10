% The reduce step of graph SLAM.
% 
% T is the number of poses and m is the number of walls
%
% Input:        omega               (T+m)x(T+m)
%               xi                  (T+m)x1
%               tau                 Txm         The poses from where each feature was observed
%               T                   1x1

function [omega_, xi_] = reduce(omega, xi, tau, T)
	wallc = (size(omega,1)-T*3)/2;
	
	omega_ = omega;
	xi_ = xi;
	
	x = @(i)     3*i-2 :     3*i;
	m = @(i) 3*T+2*i-1 : 3*T+2*i;
	
	for j=1:wallc
		for t=find(tau(:,j))
			omega_tj = omega_(x(t),m(j));
			omega_jj = omega_(m(j),m(j)) ^ -1;
			omega_jt = omega_(m(j),x(t));
			xi_j     =     xi(m(j));
			
			xi_(x(t))         =         xi_(x(t)) - omega_tj*omega_jj*xi_j;
			xi_(m(j))         =         xi_(m(j)) - omega_tj*omega_jj*xi_j;
			
			omega_(x(t),x(t)) = omega_(x(t),m(j)) - omega_tj*omega_jj*omega_jt;
			omega_(m(j),m(j)) = omega_(x(t),m(j)) - omega_tj*omega_jj*omega_jt;
			
			%omega_(x(t),m(j))*inv(omega(m(j),m(j)))*omega_(m(j),x(j))
			%omega_(x(t),m(j))*inv(omega(m(j),m(j)))*omega_(m(j),x(j))
		end
		
		tj = tj:tj+1;
		mj = 3*T+2*j-1;
		mj = mj:mj+2;
		
		omega_t_j = omega_(tj, mj);
		omega_j_j = omega_(mj, mj);
		omega_j_t = omega_(mj, tj);
		xi_j      = xi(mj);
		
		xi_(tj) = xi_(tj) - omega_t_j*inv(omega_j_j)*xi_j;
		
		
	end
	
	omega_ = omega_(1:3*T,1:3*T);
	xi_ = xi_(1:3*T);
end
