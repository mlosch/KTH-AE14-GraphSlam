% class for transforming ir distance readings into points relative a pose

classdef Transform
    properties (SetAccess = private)
		ir_offsets
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
            
			TF.ir_offsets = repmat([0 0 1]', [1 n]);
			TF.ir_offsets(1:2,:) = data.xyz(:,1:2)';
        end
        
        function m = pose_to_transform_mat(TF, pose)
            R = rotate2d(-pose(3));
            m = [R, [pose(1);pose(2)]; [0 0 1]];
        end
        
        function points = irs_to_points(TF, dists)
            points = TF.ir_offsets;
            points(1, TF.front)         = points(1, TF.front)       + dists(TF.front);
            points(1, TF.rear)          = points(1, TF.rear)        - dists(TF.rear);
            points(2, TF.left_front)    = points(2, TF.left_front)  + dists(TF.left_front);
            points(2, TF.left_rear)     = points(2, TF.left_rear)   + dists(TF.left_rear);
            points(2, TF.right_front)   = points(2, TF.right_front) - dists(TF.right_front);
            points(2, TF.right_rear)    = points(2, TF.right_rear)  - dists(TF.right_rear);
        end
        
        function points = transform_to_map(TF, poses, ir_distances)
			t = size(poses,2);
			points = zeros(2, t*6);
			for i = 1:t
				poseTF = pose_to_transform_mat(TF, poses(:,i));
				p = poseTF * irs_to_points(TF,ir_distances(:,i));
				points(:, i*6-5:i*6) = p(1:2,:);

			end
        end
    end
end
