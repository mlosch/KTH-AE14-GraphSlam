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

%map = readmatrixtable('data/dats/maze-940-980-classified.dat');
map = readmatrixtable('data/dats/maze.dat');
map = map(300:400, :);

poses = integrate_poses(map.delta_pose');
irs = filter_invalid_irs(map.ir');

disp('Instantiating Transform class');
TF = Transform('data/tf.dat');

%% Wall segmentation

disp('Segmenting walls');
correspondences = segment_walls(TF, poses, irs, FRAME_LENGTH, 1);
figure;
plot_map(poses,irs,correspondences,TF)

%% Slamming

disp('Graph Slamming');
Q = .1*.1;
R = diag([.1, .1, 3*pi/180].^2);
corrected_poses = graphSLAM(poses, irs, correspondences, R, Q, TF, 5);

%% Plotting
figure;
plot_map(corrected_poses,irs,correspondences,TF)
axis equal