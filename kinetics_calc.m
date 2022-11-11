%% Setup files
name_motion={'walking'    'jogging'   'crouch'};
name_grf   ={'walking_FP' 'jogging_FP' 'crouch_FP'};

% frame_sel  =[];

index=1; % select the motion to be loaded and visualized e.g., index=1 -> NormWalk
%% Read marker trajectory and ground reaction data
% data files should be in the same folder as the .m file
file_dir = pwd;
data_trc = readtable(fullfile(file_dir,[name_motion{index} '.txt']));
data_grf = readtable(fullfile(file_dir,[name_grf{index} '.txt']));

% Load the calculated angles
angles_left = readtable(fullfile(file_dir, 'angles_left.txt'));
angles_right = readtable(fullfile(file_dir, 'angles_right.txt'));

%% Downsample ground reaction data
% down sample the ground reaction data, so it has the same length as marker trajectory
data_grf_s = downsample(data_grf,10);

%% Variables
height = 1705; % mm
weight = 65.3; % kg

%% Ankle
m_F = 0.0145 * weight;

force_A_x = m_F * a_F_x - force_GR_x;
force_A_y = m_F * a_F_y - force_GR_y + force_foot_gravity;

moment_A = inertia_F * alfa - force_GR_perp * dist_GR + force_A_x_perp * dist_A + force_A_y_perp * dist_A;