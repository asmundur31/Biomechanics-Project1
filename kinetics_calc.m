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

%% ANKLES
% Constant for both left and right
n = 2; % We add two extra frames to the gait cycle for derivations 
toMeters = 1/1000;
timeStep = 1/100;

m_F = 0.0145 * weight; % Mass of the foot
foot_gravity_force = m_F * g;
inertia_F = m_F * (0.19^2);

%% Left ankle
% Time range
leftTimeRange_der = (288-n:386);
leftTimeRange = (288:386);

% Data from force plate 1 
FP1_force_x = data_grf_s.FP1_Force_Y;            
FP1_force_y = data_grf_s.FP1_Force_Z;

FP1_COP_x  = data_grf_s.FP1_COP_Y*toMeters;     
FP1_COP_y  = data_grf_s.FP1_COP_Z*toMeters;

% Read coordinates for left ankle
LANKLE_x = data_trc.LAJC_Y(leftTimeRange_der) * toMeters;
LANKLE_y = data_trc.LAJC_Z(leftTimeRange_der) * toMeters;

l_a_F_x = []; % Acceleration of the left foot in x-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LANKLE_x(i-2) - 2*LANKLE_x(i-1) + LANKLE_x(i)) / timeStep^2;
    l_a_F_x(i-2, 1) = a;
end

l_a_F_y = []; % Acceleration of the left foot in y-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LANKLE_y(i-2) - 2*LANKLE_y(i-1) + LANKLE_y(i)) / timeStep^2;
    l_a_F_y(i-2, 1) = a;
end

% Ground reaction forces
l_force_GR_x = FP1_force_x(leftTimeRange);
l_force_GR_y = FP1_force_y(leftTimeRange);

% Distance calculations
LANKLE_x = data_trc.LAJC_Y(leftTimeRange) * toMeters;
LANKLE_y = data_trc.LAJC_Z(leftTimeRange) * toMeters;

LTOE_x = data_trc.LTOO_Y(leftTimeRange) * toMeters;   
LTOE_y = data_trc.LTOO_Z(leftTimeRange) * toMeters;

% Left foot center of mass coordinates
l_ankleCOM_x = (LTOE_x - LANKLE_x) * 0.5 + LANKLE_x;
l_ankleCOM_y = (LANKLE_y - LTOE_y) * 0.5 + LTOE_y;

% Left foot center of pressure coordinates
l_ankleCOP_x = FP1_COP_x(leftTimeRange);
l_ankleCOP_y = FP1_COP_y(leftTimeRange);

% Distance center of pressure - center of mass
l_dist_GR_x = l_ankleCOP_x - l_ankleCOM_x;
l_dist_GR_y = l_ankleCOM_y - l_ankleCOP_y;

l_dist_GR_x(59:end) = 0; % Write about in report
l_dist_GR_y(59:end) = 0; 

% Distance LANKLE - center of mass
l_dist_A_x = l_ankleCOM_x - LANKLE_x;
l_dist_A_y = LANKLE_y - l_ankleCOM_y;

% Read left foot angle data
footAngleL = angles_left.leftFootAngleL * pi/180; % angles in radians

l_alfa = [];
for i=3:size(leftTimeRange_der, 2)
    a = (footAngleL(i-2) - 2*footAngleL(i-1) + footAngleL(i)) / timeStep^2 ;
    l_alfa(i-2, 1) = a;
end

% Calculations left ankle
l_force_A_x = m_F * l_a_F_x - l_force_GR_x;
l_force_A_y = m_F * l_a_F_y - l_force_GR_y + foot_gravity_force;

l_moment_A = inertia_F .* l_alfa - l_force_GR_x .* l_dist_GR_y - l_force_GR_y .* l_dist_GR_x + l_force_A_x .* l_dist_A_y + l_force_A_y .* l_dist_A_x;

%% Right ankle
% Time range 
rightTimeRange_der = (237-n:336);
rightTimeRange = (237:336);

% Data from force plate 2
FP2_force_x = data_grf_s.FP2_Force_Y;            
FP2_force_y = data_grf_s.FP2_Force_Z;

FP2_COP_x  = data_grf_s.FP2_COP_Y * toMeters;     
FP2_COP_y  = data_grf_s.FP2_COP_Z * toMeters;

% Read coordinates for right ankle
RANKLE_x = data_trc.RAJC_Y(rightTimeRange_der) * toMeters;
RANKLE_y = data_trc.RAJC_Z(rightTimeRange_der) * toMeters;

r_a_F_x = []; % Acceleration of the right foot in x-direction
for i=3:size(rightTimeRange_der, 2)
    a = (RANKLE_x(i-2) - 2*RANKLE_x(i-1) + RANKLE_x(i)) / timeStep^2;
    r_a_F_x(i-2, 1) = a;
end

r_a_F_y = []; % Acceleration of the right foot in y-direction
for i=3:size(rightTimeRange_der, 2)
    a = (RANKLE_y(i-2) - 2*RANKLE_y(i-1) + RANKLE_y(i)) / timeStep^2;
    r_a_F_y(i-2, 1) = a;
end

% Ground reaction forces
r_force_GR_x = FP2_force_x(rightTimeRange);
r_force_GR_y = FP2_force_y(rightTimeRange);

% Distance calculations
RANKLE_x = data_trc.RAJC_Y(rightTimeRange) * toMeters;
RANKLE_y = data_trc.RAJC_Z(rightTimeRange) * toMeters;

RTOE_x = data_trc.RTOO_Y(rightTimeRange) * toMeters;   
RTOE_y = data_trc.RTOO_Z(rightTimeRange) * toMeters;

r_ankleCOM_x = (RTOE_x - RANKLE_x) * 0.5 + RANKLE_x;
r_ankleCOM_y = (RANKLE_y - RTOE_y) * 0.5 + RTOE_y;

r_ankleCOP_x = FP2_COP_x(rightTimeRange);
r_ankleCOP_y = FP2_COP_y(rightTimeRange);

r_dist_GR_x = r_ankleCOP_x - r_ankleCOM_x;
r_dist_GR_x(59:end) = 0; % Write about in report

r_dist_GR_y = r_ankleCOM_y - r_ankleCOP_y;
r_dist_GR_y(59:end) = 0; 

r_dist_A_x = r_ankleCOM_x - RANKLE_x;
r_dist_A_y = RANKLE_y - r_ankleCOM_y;

% Read foot angle data
footAngleR = angles_right.rightFootAngleR * pi/180; % angles in radians

r_alfa = [];
for i=3:size(rightTimeRange_der, 2)
    a = (footAngleR(i-2) - 2*footAngleR(i-1) + footAngleR(i)) / timeStep^2 ;
    r_alfa(i-2, 1) = a;
end

% Calculations
r_force_A_x = m_F * r_a_F_x - r_force_GR_x;
r_force_A_y = m_F * r_a_F_y - r_force_GR_y + foot_gravity_force;

r_moment_A = inertia_F .* r_alfa - r_force_GR_x .* r_dist_GR_y - r_force_GR_y .* r_dist_GR_x + r_force_A_x .* r_dist_A_y + r_force_A_y .* r_dist_A_x;

%% Plot ankle results
timeL = linspace(0, 100, length(leftTimeRange));
timeR = linspace(0, 100, length(rightTimeRange));

subplot(2, 1, 1); % Foot segment angular acceleration
plot(timeR, r_alfa, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_alfa, 'red', 'LineWidth', 1.5);
title('Foot segment angular acceleration')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -200 200])
grid on

subplot(2, 1, 2); % Ankle moment
plot(timeR, -r_moment_A, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, -l_moment_A, 'red', 'LineWidth', 1.5);
title('Ankle moment')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Dorsiflexor - / Plantarflexor + [N*m]', 'FontSize', 9)
axis([0 100 -25 110])
grid on

%% KNEES
%% Left knee
%% Right knee
%% Plot knee results