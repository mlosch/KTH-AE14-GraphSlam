function pairs = pair_plausible_walls( correspondences, walls )
%pair_plausible_walls
%
% Inputs:   correspondences     6xt
%           walls               4xn
%
% Output:   pairs               2xO(n^2)

global MAX_ANGLE_DIFF;
global MAX_DISTANCE;

%n = max(max(correspondences));
n = size(walls,2);
pairs = zeros(2, ceil(n*n/2));

k = 1;
for i=1:n
    for j=i+1:n
        
        %evaluate if pair reasonable to choose
        Nu_i = walls(1,i);
        Nu_j = walls(1,j);
        
        c_i = walls(2,i);
        c_j = walls(2,j);
        
        Nu_diff = wrapToPi(Nu_i - Nu_j);
        
        if (Nu_diff > MAX_ANGLE_DIFF) continue; end
        
        n_i = [cos(Nu_i);sin(Nu_i)];
        n_j = [cos(Nu_j);sin(Nu_j)];
        
        offset_i = n_i*c_i;
        offset_j = n_j*c_j;
        
		orth_i = [n_i(2);-n_i(1)];
        orth_j = [n_j(2);-n_j(1)];
		
		s_i = orth_i*walls(3,i)+offset_i;
		e_i = orth_i*walls(4,i)+offset_i;
        
        s_j = orth_j*walls(3,j)+offset_j;
		e_j = orth_j*walls(4,j)+offset_j;
        
        dist = norm((s_i+e_i)/2 - (s_j+e_j)/2);
        
        if (dist > MAX_DISTANCE) continue; end
        
        %accept pair
        pairs(1,k) = i;
        pairs(2,k) = j;
        k = k+1;
    end
end

pairs(:,k:end) = [];


end

