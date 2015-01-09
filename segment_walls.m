function [ correspondences ] = segment_walls( TF, poses, irs, framelen, skipFrontRearIR )
%SEGMENT_PLANES Determines walls in given measurement cloud
% Inputs:       TF                  Transform instance
%               poses               3xt
%               irs                 6xt
%               framelen            1x1 Timeframe length in s
%               skipFrontRearIR     1x1
%
% Output:       correspondences     6xt

if (nargin < 5)
    skipFrontRearIR = 1;
end

global HALT_CONDITION;
global MAX_DIST_TO_HYPOTHESIS;
global MIN_NUM_OF_INLIERS;
global RANSAC_ITERATIONS;

dt = 0.1; % Time between two consecutive (pose,measurement)
dn = framelen / dt; % poses per frame
N = size(poses,2);

correspondences = zeros(6,N);

wallid = 1;

points = TF.transform_to_map(poses, irs);

i = 1;
while i < N && (i+dn) <= N
    
    %gather valid measurements and transform them to world coordinates
    
    %indices = zeros(nIR*dn,1);
    indices = ((i-1)*6+1):(((i-1)+dn)*6);
    
    if (skipFrontRearIR == 1)
        del_ind = [1:6:length(indices) 2:6:length(indices)];
        indices(del_ind) = [];
    end
    
    %remove indices that point to NaN
    indices(isnan(points(indices))==1) = [];
    
    %perform ransac on points until a minimum ratio of points is assigned
    nInliers = 0;
    nPoints = size(indices,2);
    
    while( nInliers/nPoints < HALT_CONDITION )
        
        [t, r, inliers] = ransac(points, indices, RANSAC_ITERATIONS, MAX_DIST_TO_HYPOTHESIS, MIN_NUM_OF_INLIERS);
        
        if (isempty(inliers))
            %disp('Could not fit more lines into point set');
            break;
        end
        
%         plot(points(1,:),points(2,:),'ko');
%         hold on;
%         plot(points(1,indices),points(2,indices),'go');
%         plot(points(1,inliers),points(2,inliers),'ro');
% %         X = 0:0.1:2;%-N/20:N/20;
% %         k1 = -tan(t);
% %         b1 = r/cos(t);
% %         plot(X,k1*X+b1,'r');
%         axis equal;
%         hold off;
%         pause;
        
        nInliers = nInliers + length(inliers);
        
        %fill correspondences
        for k=1:length(inliers) 
           [row, col] = get_array_idx(inliers(k),6); 
           correspondences(col,row) = wallid;
        end
        
        wallid = wallid+1;
        
        indices(ismember(indices,inliers)) = [];
        
    end
    
    i = i+dn;
    
end
end