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

if index==1
    angles_left = readtable(fullfile(file_dir, 'walking_angles_left.txt'));
    angles_right = readtable(fullfile(file_dir, 'walking_angles_right.txt'));
elseif index==2
    angles_left = readtable(fullfile(file_dir, 'jogging_angles_left.txt'));
    angles_right = readtable(fullfile(file_dir, 'jogging_angles_right.txt'));
end

%% Constants
height = 1705; % mm
weight = 65.3; % kg
g = 9.818; % m/s^2 
n = 2; % We add two extra frames to the gait cycle for derivations 
toMeters = 1/1000;
timeStep = 1/100;

% Time range
if index==1
    leftTimeRange_der = (288-n:386);
    leftTimeRange = (288:386);
    
    rightTimeRange_der = (237-n:336);
    rightTimeRange = (237:336);
elseif index==2
    leftTimeRange_der = (163-n:229);
    leftTimeRange = (163:229);
    
    rightTimeRange_der = (163-n:229);
    rightTimeRange = (163:229);
end

%% ANKLES
% Constant for both left and right
m_F = 0.0145 * weight; % Mass of the foot
foot_gravity_force = m_F * g;
inertia_F = m_F * (0.69-0.475)^2;

%% Left ankle
% Data from force plate 1 
FP1_force_x = data_grf_s.FP1_Force_Y;            
FP1_force_y = data_grf_s.FP1_Force_Z;

FP1_COP_x  = data_grf_s.FP1_COP_Y*toMeters;     
FP1_COP_y  = data_grf_s.FP1_COP_Z*toMeters;

% Read coordinates for left ankle
LANKLE_x = data_trc.LAJC_Y(leftTimeRange_der) * toMeters;
LANKLE_y = data_trc.LAJC_Z(leftTimeRange_der) * toMeters;

LTOE_x = data_trc.LTOO_Y(leftTimeRange_der) * toMeters;   
LTOE_y = data_trc.LTOO_Z(leftTimeRange_der) * toMeters;

% Read left foot angle data
footAngleL = angles_left.leftFootAngleL * pi/180; % angles in radians

% Ground reaction forces
l_force_GR_x = FP1_force_x(leftTimeRange);
l_force_GR_y = FP1_force_y(leftTimeRange);

% Distance calculations
% Left foot center of mass coordinates
l_footCOM_x = (LTOE_x - LANKLE_x) * 0.5 + LANKLE_x;
l_footCOM_y = (LANKLE_y - LTOE_y) * 0.5 + LTOE_y;

% Left foot center of pressure coordinates
l_footCOP_x = FP1_COP_x(leftTimeRange_der);
l_footCOP_y = FP1_COP_y(leftTimeRange_der);

% Distance center of pressure - center of mass
l_dist_GR_x = l_footCOP_x - l_footCOM_x;
l_dist_GR_y = l_footCOM_y - l_footCOP_y;

l_dist_GR_x(59:end) = 0; % Write about in report
l_dist_GR_y(59:end) = 0; 

% Distance LANKLE - center of mass
l_dist_AF_x = l_footCOM_x - LANKLE_x;
l_dist_AF_y = LANKLE_y - l_footCOM_y;

% Acceleration calculations
l_a_F_x = []; % Acceleration of the left foot in x-direction
for i=3:size(leftTimeRange_der, 2)
    a = (l_footCOM_x(i-2) - 2*l_footCOM_x(i-1) + l_footCOM_x(i)) / timeStep^2;
    l_a_F_x(i-2, 1) = a;
end

l_a_F_y = []; % Acceleration of the left foot in y-direction
for i=3:size(leftTimeRange_der, 2)
    a = (l_footCOM_y(i-2) - 2*l_footCOM_y(i-1) + l_footCOM_y(i)) / timeStep^2;
    l_a_F_y(i-2, 1) = a;
end

l_alfa_f = [];
for i=3:size(leftTimeRange_der, 2)
    a = (footAngleL(i-2) - 2*footAngleL(i-1) + footAngleL(i)) / timeStep^2 ;
    l_alfa_f(i-2, 1) = a;
end

% Calculations left ankle
l_force_A_x = m_F * l_a_F_x - l_force_GR_x;
l_force_A_y = m_F * l_a_F_y - l_force_GR_y + foot_gravity_force;

l_moment_A = inertia_F .* l_alfa_f - l_force_GR_x .* l_dist_GR_y(n+1:end) - l_force_GR_y .* l_dist_GR_x(n+1:end) + l_force_A_x .* l_dist_AF_y(n+1:end) + l_force_A_y .* l_dist_AF_x(n+1:end);

l_power_A = l_moment_A .* l_alfa_f;

%% Right ankle
% Data from force plate 2
FP2_force_x = data_grf_s.FP2_Force_Y;            
FP2_force_y = data_grf_s.FP2_Force_Z;

FP2_COP_x  = data_grf_s.FP2_COP_Y * toMeters;     
FP2_COP_y  = data_grf_s.FP2_COP_Z * toMeters;

% Read coordinates for right ankle
RANKLE_x = data_trc.RAJC_Y(rightTimeRange_der) * toMeters;
RANKLE_y = data_trc.RAJC_Z(rightTimeRange_der) * toMeters;

RTOE_x = data_trc.RTOO_Y(rightTimeRange_der) * toMeters;   
RTOE_y = data_trc.RTOO_Z(rightTimeRange_der) * toMeters;

% Read foot angle data
footAngleR = angles_right.rightFootAngleR * pi/180; % angles in radians

% Ground reaction forces
r_force_GR_x = FP2_force_x(rightTimeRange);
r_force_GR_y = FP2_force_y(rightTimeRange);

% Distance calculations
r_footCOM_x = (RTOE_x - RANKLE_x) * 0.5 + RANKLE_x;
r_footCOM_y = (RANKLE_y - RTOE_y) * 0.5 + RTOE_y;

r_footCOP_x = FP2_COP_x(rightTimeRange_der);
r_footCOP_y = FP2_COP_y(rightTimeRange_der);

r_dist_GR_x = r_footCOP_x - r_footCOM_x;
r_dist_GR_y = r_footCOM_y - r_footCOP_y;

r_dist_GR_x(59:end) = 0; % Write about in report
r_dist_GR_y(59:end) = 0; 

r_dist_AF_x = r_footCOM_x - RANKLE_x;
r_dist_AF_y = RANKLE_y - r_footCOM_y;

% Calculate acceleration
r_a_F_x = []; % Acceleration of the right foot in x-direction
for i=3:size(rightTimeRange_der, 2)
    a = (r_footCOM_x(i-2) - 2*r_footCOM_x(i-1) + r_footCOM_x(i)) / timeStep^2;
    r_a_F_x(i-2, 1) = a;
end

r_a_F_y = []; % Acceleration of the right foot in y-direction
for i=3:size(rightTimeRange_der, 2)
    a = (r_footCOM_y(i-2) - 2*r_footCOM_y(i-1) + r_footCOM_y(i)) / timeStep^2;
    r_a_F_y(i-2, 1) = a;
end

r_alfa_f = [];
for i=3:size(rightTimeRange_der, 2)
    a = (footAngleR(i-2) - 2*footAngleR(i-1) + footAngleR(i)) / timeStep^2 ;
    r_alfa_f(i-2, 1) = a;
end

% Calculations
r_force_A_x = m_F * r_a_F_x - r_force_GR_x;
r_force_A_y = m_F * r_a_F_y - r_force_GR_y + foot_gravity_force;

r_moment_A = inertia_F .* r_alfa_f - r_force_GR_x .* r_dist_GR_y(n+1:end) - r_force_GR_y .* r_dist_GR_x(n+1:end)  + r_force_A_x .* r_dist_AF_y(n+1:end)  + r_force_A_y .* r_dist_AF_x(n+1:end) ;

r_power_A = r_moment_A .* r_alfa_f;

%% Plot ankle results
timeL = linspace(0, 100, length(leftTimeRange));
timeR = linspace(0, 100, length(rightTimeRange));

figure(1)
subplot(3, 1, 1); % Foot segment angular acceleration
plot(timeR, r_alfa_f, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_alfa_f, 'red', 'LineWidth', 1.5);
title('Foot segment angular acceleration')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -200 200])
grid on

subplot(3, 1, 2); % Ankle moment
plot(timeR, -r_moment_A, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, -l_moment_A, 'red', 'LineWidth', 1.5);
title('Ankle moment')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Dorsiflexor - / Plantarflexor + [N*m]', 'FontSize', 9)
axis([0 100 -25 110])
grid on

subplot(3, 1, 3); % Ankle power
plot(timeR, r_power_A, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_power_A, 'red', 'LineWidth', 1.5);
title('Ankle power')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
grid on

%% KNEES
% Constant for both left and right
m_S = 0.0465 * weight; % Mass of the shank
inertia_S = m_S * (0.528 - 0.302)^2;
shank_gravity_force = m_S * g;

%% Left knee
% Read coordinates for left knee
LKNEE_x = data_trc.LKJC_Y(leftTimeRange_der) * toMeters;
LKNEE_y = data_trc.LKJC_Z(leftTimeRange_der) * toMeters;

% Calculate distances
l_shankCOM_x = (LKNEE_x - LANKLE_x) * 0.567 + LANKLE_x;
l_shankCOM_y = (LKNEE_y - LANKLE_y) * 0.567 + LANKLE_y;

% Distance center of mass - ankle
l_dist_AS_x = l_shankCOM_x - LANKLE_x;
l_dist_AS_y = l_shankCOM_y - LANKLE_y;

% Distance center of mass - knee
l_dist_KS_x = LKNEE_x - l_shankCOM_x;
l_dist_KS_y = LKNEE_y - l_shankCOM_y; 

% Calculate acceleration of shank COM
l_a_K_x = []; % Acceleration of the left shank in x-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LKNEE_x(i-2) - 2*LKNEE_x(i-1) + LKNEE_x(i)) / timeStep^2;
    l_a_K_x(i-2, 1) = a;
end

l_a_K_y = []; % Acceleration of the left shank in y-direction
for i=3:size(leftTimeRange_der, 2)
    a = (LKNEE_y(i-2) - 2*LKNEE_y(i-1) + LKNEE_y(i)) / timeStep^2;
    l_a_K_y(i-2, 1) = a;
end

% Read left shank angle data
shankAngleL = angles_left.leftShankAngleL * pi/180; % angles in radians

l_alfa_S = [];
for i=3:size(leftTimeRange_der, 2)
    a = (shankAngleL(i-2) - 2*shankAngleL(i-1) + shankAngleL(i)) / timeStep^2 ;
    l_alfa_S(i-2, 1) = a;
end

% Calculations
l_force_K_x = m_S * l_a_K_x + l_force_A_x;
l_force_K_y = m_S * l_a_K_y + l_force_A_y + shank_gravity_force;

l_moment_K = inertia_S .* l_alfa_S + l_moment_A + l_force_A_x .* l_dist_AS_y(n+1:end) - l_force_A_y .* l_dist_AS_x(n+1:end) - l_force_K_y .* l_dist_KS_x(n+1:end) + l_force_K_x .* l_dist_KS_y(n+1:end);

l_power_K = l_moment_K .* l_alfa_S;

%% Right knee
% Read coordinates for right knee
RKNEE_x = data_trc.RKJC_Y(rightTimeRange_der) * toMeters;
RKNEE_y = data_trc.RKJC_Z(rightTimeRange_der) * toMeters;

% Calculate distances
r_shankCOM_x = (RKNEE_x - RANKLE_x) * 0.567 + RANKLE_x;
r_shankCOM_y = (RKNEE_y - RANKLE_y) * 0.567 + RANKLE_y;

% Distance center of mass - ankle
r_dist_AS_x = r_shankCOM_x - RANKLE_x;
r_dist_AS_y = r_shankCOM_y - RANKLE_y;

% Distance center of mass - knee
r_dist_KS_x = RKNEE_x - r_shankCOM_x;
r_dist_KS_y = RKNEE_y - r_shankCOM_y; 

r_a_S_x = []; % Acceleration of the right shank in x-direction
for i=3:size(rightTimeRange_der, 2)
    a = (RKNEE_x(i-2) - 2*RKNEE_x(i-1) + RKNEE_x(i)) / timeStep^2;
    r_a_S_x(i-2, 1) = a;
end

r_a_S_y = []; % Acceleration of the right shank in y-direction
for i=3:size(rightTimeRange_der, 2)
    a = (RKNEE_y(i-2) - 2*RKNEE_y(i-1) + RKNEE_y(i)) / timeStep^2;
    r_a_S_y(i-2, 1) = a;
end

% Read left shank angle data
shankAngleR = angles_right.rightShankAngleR * pi/180; % angles in radians

r_alfa_S = [];
for i=3:size(rightTimeRange_der, 2)
    a = (shankAngleR(i-2) - 2*shankAngleR(i-1) + shankAngleR(i)) / timeStep^2 ;
    r_alfa_S(i-2, 1) = a;
end

% Calculations
r_force_K_x = m_S * r_a_S_x + r_force_A_x;
r_force_K_y = m_S * r_a_S_y + r_force_A_y + shank_gravity_force;

r_moment_K = inertia_S .* r_alfa_S + r_moment_A + r_force_A_x .* r_dist_AS_y(n+1:end) - r_force_A_y .* r_dist_AS_x(n+1:end) - r_force_K_y .* r_dist_KS_x(n+1:end) + r_force_K_x .* r_dist_KS_y(n+1:end);

r_power_K = r_moment_K .* r_alfa_S;

%% Plot knee results
figure(2)
subplot(3, 1, 1); % Shank segment angular acceleration
plot(timeR, r_alfa_S, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_alfa_S, 'red', 'LineWidth', 1.5);
title('Shank segment angular acceleration')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
grid on

subplot(3, 1, 2); % Knee moment
plot(timeR, r_moment_K, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_moment_K, 'red', 'LineWidth', 1.5);
title('Knee moment')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Hip flexor - / Hip extensor + [N*m]', 'FontSize', 9)
grid on

subplot(3, 1, 3); % Knee power
plot(timeR, r_power_K, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_power_K, 'red', 'LineWidth', 1.5);
title('Knee power')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
grid on

%% HIPS
% Constant for both left and right
m_T = 0.1 * weight;
hip_gravity_force = m_T * g;
inertia_T = m_T * (0.323 - 0.54)^2;

%% Left hip
% Read coordinates for left hip
LHIP_x=data_trc.LHJC_Y(leftTimeRange_der)*toMeters;   
LHIP_y=data_trc.LHJC_Z(leftTimeRange_der)*toMeters; 

% Calculate distances
l_thighCOM_x = (LHIP_x - LKNEE_x) * 0.567 + LKNEE_x;
l_thighCOM_y = (LHIP_y - LKNEE_y) * 0.567 + LKNEE_y;

% Distance center of mass - knee
l_dist_kt_x = l_thighCOM_x - LKNEE_x;
l_dist_kt_y = l_thighCOM_y - LKNEE_y;

% Distance center of mass - hip
l_dist_ht_x = LHIP_x - l_thighCOM_x;
l_dist_ht_y = LHIP_y - l_thighCOM_y; 

l_a_T_x = []; % Acceleration of the left thigh in x-direction
for i=3:size(leftTimeRange_der, 2)
    a = (l_thighCOM_x(i-2) - 2*l_thighCOM_x(i-1) + l_thighCOM_x(i)) / timeStep^2;
    l_a_T_x(i-2, 1) = a;
end

l_a_T_y = []; % Acceleration of the left thigh in y-direction
for i=3:size(leftTimeRange_der, 2)
    a = (l_thighCOM_y(i-2) - 2*l_thighCOM_y(i-1) + l_thighCOM_y(i)) / timeStep^2;
    l_a_T_y(i-2, 1) = a;
end

% Read left thigh angle data
thighAngleL = angles_left.leftThighAngleL * pi/180; % angles in radians

l_alfa_T = [];
for i=3:size(leftTimeRange_der, 2)
    a = (thighAngleL(i-2) - 2*thighAngleL(i-1) + thighAngleL(i)) / timeStep^2 ;
    l_alfa_T(i-2, 1) = a;
end

% Calculations
l_force_H_x = m_T * l_a_T_x + l_force_K_x;
l_force_H_y = m_T * l_a_T_y + l_force_K_y + hip_gravity_force;

l_moment_H = inertia_T .* l_alfa_T + l_moment_K + l_force_K_x .* l_dist_kt_y(n+1:end) + l_force_K_y .* l_dist_kt_x(n+1:end) + l_force_H_y .* l_dist_ht_x(n+1:end) + l_force_H_x .* l_dist_ht_y(n+1:end);

l_power_H = l_moment_H .* l_alfa_T;

%% Right hip
% Read coordinates for right hip
RHIP_x = data_trc.RHJC_Y(rightTimeRange_der)*toMeters;   
RHIP_y = data_trc.RHJC_Z(rightTimeRange_der)*toMeters; 

% Calculate distances
r_thighCOM_x = (RHIP_x - RKNEE_x) * 0.567 + RKNEE_x;
r_thighCOM_y = (RHIP_y - RKNEE_y) * 0.567 + RKNEE_y;

% Distance center of mass - knee
r_dist_kt_x = r_thighCOM_x - RKNEE_x;
r_dist_kt_y = r_thighCOM_y - RKNEE_y;

% Distance center of mass - hip
r_dist_ht_x = RHIP_x - r_thighCOM_x;
r_dist_ht_y = RHIP_y - r_thighCOM_y; 

r_a_T_x = []; % Acceleration of the right thigh in x-direction
for i=3:size(rightTimeRange_der, 2)
    a = (r_thighCOM_x(i-2) - 2*r_thighCOM_x(i-1) + r_thighCOM_x(i)) / timeStep^2;
    r_a_T_x(i-2, 1) = a;
end

r_a_T_y = []; % Acceleration of the right thigh in y-direction
for i=3:size(rightTimeRange_der, 2)
    a = (r_thighCOM_y(i-2) - 2*r_thighCOM_y(i-1) + r_thighCOM_y(i)) / timeStep^2;
    r_a_T_y(i-2, 1) = a;
end

% Read right thigh angle data
thighAngleR = angles_right.rightThighAngleR * pi/180; % angles in radians

r_alfa_T = [];
for i=3:size(rightTimeRange_der, 2)
    a = (thighAngleR(i-2) - 2*thighAngleR(i-1) + thighAngleR(i)) / timeStep^2 ;
    r_alfa_T(i-2, 1) = a;
end

% Calculations
r_force_H_x = m_T * r_a_T_x + r_force_K_x;
r_force_H_y = m_T * r_a_T_y + r_force_K_y + hip_gravity_force;

r_moment_H = inertia_T .* r_alfa_T + r_moment_K + r_force_K_x .* r_dist_kt_y(n+1:end) + r_force_K_y .* r_dist_kt_x(n+1:end) + r_force_H_y .* r_dist_ht_x(n+1:end) + r_force_H_x .* r_dist_ht_y(n+1:end);

r_power_H = r_moment_H .* r_alfa_T;

%% Plot hip results
figure(3)
subplot(3, 1, 1); % Thigh segment angular acceleration
plot(timeR, r_alfa_T, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_alfa_T, 'red', 'LineWidth', 1.5);
title('Thigh segment angular acceleration')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -40 30])
grid on

subplot(3, 1, 2); % Hip moment
plot(timeR, r_moment_H, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_moment_H, 'red', 'LineWidth', 1.5);
title('Hip moment')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Hip flexor - / Hip extensor + [N*m]', 'FontSize', 9)
%axis([0 100 -25 110])
grid on

subplot(3, 1, 3); % Hip power
plot(timeR, r_power_H, 'green', 'LineWidth', 1.5);
hold on
plot(timeL, l_power_H, 'red', 'LineWidth', 1.5);
title('Hip power')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
grid on

%% SAVE DATA
if index == 1
    walking_table = table(r_alfa_f, r_moment_A, r_power_A, r_alfa_S, r_moment_K, r_power_K, r_alfa_T, r_moment_H, r_power_H);
    writetable(walking_table,'walking_kinetics.txt', 'Delimiter',' ')
elseif index == 2
    jogging_table = table(r_alfa_f, r_moment_A, r_power_A, r_alfa_S, r_moment_K, r_power_K, r_alfa_T, r_moment_H, r_power_H);
    writetable(jogging_table,'jogging_kinetics.txt', 'Delimiter',' ')
end