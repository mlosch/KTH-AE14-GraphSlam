classdef Transform
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        tomap_irs
    end
    
    properties
        front = 1
        rear = 2
        left_front = 3
        left_rear = 4
        right_front = 5
        right_rear = 6
    end
    
    methods
        function TF = Transform(tf_dat)
            data = readmatrixtable(tf_dat,'ReadRowNames',true);
            
            n = size(data,1);
            TF.tomap_irs = zeros(n,3,3);
            
            for i=1:n
                R = rotate2d(data.rpy(i,3));
                TF.tomap_irs(i,:,:) = [R, data.xyz(i,1:2)'; [0 0 1]];
            end
        end
        
        function m = pose_to_transform_mat(TF, pose)
            R = rotate2d(pose(3));
            m = [R, [pose(1);pose(2)]; [0 0 1]];
        end
        
        function points = irs_to_points(TF,dists)
            points = ones(length(dists),3)*diag([0 0 1]);
            points(TF.front,1)          = dists(TF.front);
            points(TF.rear,1)           = -dists(TF.rear);
            points(TF.left_front,2)     = dists(TF.left_front);
            points(TF.left_rear,2)      = dists(TF.left_rear);
            points(TF.right_front,2)    = -dists(TF.right_front);
            points(TF.right_rear,2)     = -dists(TF.right_rear);
            points = points';
        end
        
        function points = transform_to_map(TF, pose, ir_distances)
            points_ir = irs_to_points(TF,ir_distances);
            
            n = size(TF.tomap_irs,1);
            points = zeros(3,n);
            
            poseTF = pose_to_transform_mat(TF, pose);
            
            for i=1:size(TF.tomap_irs,1)
                points(:,i) = poseTF * reshape(TF.tomap_irs(i,:,:),3,3) * points_ir(:,i);
            end
            
            points = points(1:2,:);
        end
        
		function points = transform_to_map_multiple(TF, poses, ir_distances)
			t = size(poses,1);
			points = zeros(2, t*6);
			for i = 1:t
				some_points = TF.transform_to_map(poses(i,:), ir_distances(i,:));
				points(:, i*6-5:i*6) = some_points;
			end
		end
		
    end
    
end

