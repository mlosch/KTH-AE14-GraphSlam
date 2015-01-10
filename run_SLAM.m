clear all;

%% Constants
% For Wall segmentation:
global HALT_CONDITION; HALT_CONDITION = 0.9;
global MAX_DIST_TO_HYPOTHESIS; MAX_DIST_TO_HYPOTHESIS = 0.05;
global MIN_NUM_OF_INLIERS; MIN_NUM_OF_INLIERS = 8;
global FRAME_LENGTH; FRAME_LENGTH = 1.0;
global RANSAC_ITERATIONS; RANSAC_ITERATIONS = 50;

% Script related:

%% Init
disp('Reading poses and ir data');
%maze_data=readmatrixtable('data/dats/maze-940-980-classified.dat');
maze_data = readmatrixtable('data/dats/maze.dat');

disp('Preparing data');
startidx = 300;
numPoses = 1000;
poses = maze_data.pose(startidx:startidx+numPoses,:)';
poses(1:2,:) = poses(1:2,:) - repmat(poses(1:2,1), [1 size(poses,2)]); %make poses start at (0,0,theta)
%poses(3,:) = wrapToPi(poses(3,:)); 
irs = maze_data.ir(startidx:startidx+numPoses,:)';

%poses = maze_data.pose';
%irs = maze_data.ir';

%filter invalid ir readings
irs(irs > 0.6 | irs <= 0.0) = NaN;

disp('Instantiating Transform class');
TF = Transform('data/tf.dat');

%% Wall segmentation

disp('Segmenting walls');
correspondences = segment_walls(TF, poses, irs, FRAME_LENGTH, 1);

%% Slamming

disp('Graph Slamming');
poses = graphSLAM(poses, irs, correspondences, R, Q, TF, 5);

%% Plotting
plot_map(poses,irs,correspondences,TF)
axis equal