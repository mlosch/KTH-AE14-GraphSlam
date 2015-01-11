function map = run_SLAM(varargin)

%% Constants
% For Wall segmentation:
global HALT_CONDITION; HALT_CONDITION = 0.9;
global MAX_DIST_TO_HYPOTHESIS; MAX_DIST_TO_HYPOTHESIS = 0.05;
global MIN_NUM_OF_INLIERS; MIN_NUM_OF_INLIERS = 15;
global FRAME_LENGTH; FRAME_LENGTH = 1.5;
global RANSAC_ITERATIONS; RANSAC_ITERATIONS = 100;

% For pairing walls:
global MAX_ANGLE_DIFF; MAX_ANGLE_DIFF = deg2rad(20.0);
global MAX_DISTANCE; MAX_DISTANCE = 0.1;

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
	map = map(330:400, :);
	
	fprintf(' (%.1f s)\n', toc);
end

delta_poses = map.delta_pose';
poses = integrate_poses(delta_poses);
irs = filter_invalid_irs(map.ir');

TF = Transform('data/tf.dat');

%% Wall segmentation
fprintf('Segmenting walls'), tic;

correspondences = segment_walls(TF, poses, irs, FRAME_LENGTH, 1);
figure;
plot_map(poses,irs,correspondences,TF);
walls = fit_walls(poses, irs, correspondences, TF);
pairs = pair_plausible_walls(correspondences,walls)
plot_walls(walls(:,pairs(:)),'-r');

fprintf(' (%.1f s)\n', toc);
%% Slamming
fprintf('Graph Slamming\n'), tic;

Q = .1*.1;
R = diag([.03, .03, 3*pi/180].^2);
%R=R/100;
figure;
corrected_poses = graphSLAM(poses, delta_poses, irs, correspondences, R, Q, TF, 1);

fprintf('(%.1f s)\n', toc);
%% Plotting

%figure;
%plot_map(corrected_poses,irs,correspondences,TF)
%axis equal
end
