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

%% Downsample ground reaction data
% down sample the ground reaction data, so it has the same length as marker trajectory
data_grf_s = downsample(data_grf,10);

%% Read angle data
% Load the calculated angles
angles_left = readtable(fullfile(file_dir, 'angles_left.txt'));
angles_right = readtable(fullfile(file_dir, 'angles_right.txt'));

%% Constants
height = 1705; % mm
weight = 65.3; % kg
g = 9.818; % m/s^2 

%% Left ankle
% Time range
n = 2; % We add two extra frames to the gait cycle for derivations 
leftTimeRange_der = (288-n:386);
leftTimeRange = (288:386);

toMeters = 1/1000;

% Data from force plate 1 
FP1_force_x = data_grf_s.FP1_Force_Y;            
FP1_force_y = data_grf_s.FP1_Force_Z;

FP1_COP_x  = data_grf_s.FP1_COP_Y*toMeters;     
FP1_COP_y  = data_grf_s.FP1_COP_Z*toMeters;

% Read coordinates for left ankle
LANKLE_x = data_trc.LAJC_Y(leftTimeRange_der) * toMeters;
LANKLE_y = data_trc.LAJC_Z(leftTimeRange_der) * toMeters;

m_F = 0.0145 * weight; % Mass of the foot

timeStep = 1/100;

a_F_x = []; % Acceleration of the foot in x-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LANKLE_x(i-2) - 2*LANKLE_x(i-1) + LANKLE_x(i)) / timeStep^2;
    a_F_x(i-2, 1) = a;
end

force_GR_x = FP1_force_x(leftTimeRange);

a_F_y = []; % Acceleration of the foot in y-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LANKLE_y(i-2) - 2*LANKLE_y(i-1) + LANKLE_y(i)) / timeStep^2;
    a_F_y(i-2, 1) = a;
end

force_GR_y = FP1_force_y(leftTimeRange);
force_foot_gravity = m_F * g;

inertia_F = m_F * (0.19^2);

% Distance calculations
LANKLE_x = data_trc.LAJC_Y(leftTimeRange) * toMeters;
LANKLE_y = data_trc.LAJC_Z(leftTimeRange) * toMeters;

LTOE_x = data_trc.LTOO_Y(leftTimeRange) * toMeters;   
LTOE_y = data_trc.LTOO_Z(leftTimeRange) * toMeters;

ankleCOM_x = (LTOE_x - LANKLE_x) * 0.5 + LANKLE_x;
ankleCOM_y = (LANKLE_y - LTOE_y) * 0.5 + LTOE_y;

ankleCOP_x = FP1_COP_x(leftTimeRange);
ankleCOP_y = FP1_COP_y(leftTimeRange);

dist_GR_x = ankleCOP_x - ankleCOM_x;
dist_GR_y = ankleCOM_y - ankleCOP_y;

dist_A_x = ankleCOM_x - LANKLE_x;
dist_A_y = LANKLE_y - ankleCOM_y;

dist_GR_x(59:end) = 0; % Write about in report
dist_GR_y(59:end) = 0; 

% Read foot angle data
footAngleL = angles_left.leftFootAngleL * pi/180; % angles in radians

alfa = [];
for i=3:size(leftTimeRange_der, 2)
    a = (footAngleL(i-2) - 2*footAngleL(i-1) + footAngleL(i)) / timeStep^2 ;
    alfa(i-2, 1) = a;
end

% Calculations
force_A_x = m_F * a_F_x - force_GR_x;
force_A_y = m_F * a_F_y - force_GR_y + force_foot_gravity;

moment_A = inertia_F .* alfa - force_GR_x .* dist_GR_y - force_GR_y .* dist_GR_x + force_A_x .* dist_A_y + force_A_y .* dist_A_x;

% Plot results
timeL = linspace(0, 100, length(leftTimeRange));

plot(timeL, -moment_A, 'red');
axis([0 100 -25 110])

%% Right ankle
