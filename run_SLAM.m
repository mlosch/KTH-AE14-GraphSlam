%% Constants
% For Wall segmentation:
HALT_CONDITION = 0.9;
MAX_DIST_TO_HYPOTHESIS = 0.1;
MIN_NUM_OF_INLIERS = 5;
FRAME_LENGTH = 2.0;

% Script related:

%% Init
disp('Reading poses and ir data');
maze_data=readmatrixtable('data/dats/maze.dat');
poses = maze_data.delta_pose(274:274+100,:);
irs = maze_data.ir(274:274+100,:);

disp('Instantiating Transform class');
TF = Transform('data/tf.dat');

%% Wall segmentation
disp('Segmenting walls');
correspondences = segment_walls(TF, poses, irs, FRAME_LENGTH, 1);