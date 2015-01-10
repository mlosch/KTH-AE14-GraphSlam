function run_SLAM(varargin)

%% Constants
% For Wall segmentation:
global HALT_CONDITION; HALT_CONDITION = 0.9;
global MAX_DIST_TO_HYPOTHESIS; MAX_DIST_TO_HYPOTHESIS = 0.05;
global MIN_NUM_OF_INLIERS; MIN_NUM_OF_INLIERS = 8;
global FRAME_LENGTH; FRAME_LENGTH = 1.0;
global RANSAC_ITERATIONS; RANSAC_ITERATIONS = 50;

% Script related:

%% Init
if ~isempty(varargin)
	if isa(varargin{1}, 'table')
		map = varargin{1};
	else
		error('argument not a table');
	end
else
	fprintf('Reading poses and ir data'), tic;

	%map = readmatrixtable('data/dats/maze-940-980-classified.dat');
	map = readmatrixtable('data/dats/maze.dat');
	map = map(300:400, :);
	
	fprintf(' (%.1f s)\n', toc);
end

poses = integrate_poses(map.delta_pose');
irs = filter_invalid_irs(map.ir');

TF = Transform('data/tf.dat');

%% Wall segmentation
fprintf('Segmenting walls'), tic;

correspondences = segment_walls(TF, poses, irs, FRAME_LENGTH, 1);
figure;
plot_map(poses,irs,correspondences,TF)

fprintf(' (%.1f s)\n', toc);
%% Slamming
fprintf('Graph Slamming\n'), tic;

Q = .1*.1;
R = diag([.1, .1, 3*pi/180].^2);
corrected_poses = graphSLAM(poses, irs, correspondences, R, Q, TF, 5);

fprintf('(%.1f s)\n', toc);
%% Plotting

figure;
plot_map(corrected_poses,irs,correspondences,TF)
axis equal
end
