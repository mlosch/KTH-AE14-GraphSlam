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

dt = 0.1; % Time between two consecutive (pose,measurement)
dn = framelen / dt; % poses per frame
N = size(poses,2);

correspondences = -ones(6,N);

if (skipFrontRearIR == 1)
    nIR = 4;
    ir_i = 3;
else
    nIR = 6;
    ir_i = 1;
end

wallid = 1;

pose = poses(:,1);

i = 1;
while i < N
    
    points = zeros(2, nIR*dn);
    j = 1;
    for k=i:i+dn
        
        pose = pose + poses(:,k);
        ir_points = TF.transform_to_map(pose, irs(:,k));
        points(:, j:(j+nIR)) = ir_points(:,ir_i:6);
        
        j = j+nIR;
    end
    
    %perform ransac on points until a minimum ratio of points is assigned
    nInliers = 0;
    nPoints = size(points,2);
    
    while( nInliers/nPoints < HALT_CONDITION )
        
        iterNum = 10;
        [~, ~, inliers] = ransac(points, iterNum, MAX_DIST_TO_HYPOTHESIS, MIN_NUM_OF_INLIERS/nPoints);
        
        if (isempty(inliers))
            disp('Could not fit more lines into point set');
            break;
        end
        
        nInliers = nInliers + length(inliers);
        
        %fill correspondences
        for k=1:length(inliers)
           [row col] = get_array_idx(inliers(k),nIR); 
           correspondences(col,row) = wallid;
        end
        
        wallid = wallid+1;
        
        points(:,inliers) = [];
        
    end
    
    i = i+dn;
    
end
end