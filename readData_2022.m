%%% Basic read data and visualization for the course SG2804  
%%% Biomechanics of Human Movement at KTH - 2022
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

%% Assign the uploaded table to variables in MATLAB
toMeters=1/1000; % data is originally in mm, it has to be divided by 1000 to have it in meters

RTOE_x=data_trc.RTOO_Y*toMeters;   RTOE_y=data_trc.RTOO_Z*toMeters;    % horizontal coordinate of the right toe & vertical coordinate of the right toe 
LTOE_x=data_trc.LTOO_Y*toMeters;   LTOE_y=data_trc.LTOO_Z*toMeters;    % horizontal coordinate of the left toe & vertical coordinate of the left toe 

RANKLE_x=data_trc.RAJC_Y*toMeters;   RANKLE_y=data_trc.RAJC_Z*toMeters;   
LANKLE_x=data_trc.LAJC_Y*toMeters;   LANKLE_y=data_trc.LAJC_Z*toMeters;   

RKNEE_x=data_trc.RKJC_Y*toMeters;   RKNEE_y=data_trc.RKJC_Z*toMeters;    
LKNEE_x=data_trc.LKJC_Y*toMeters;   LKNEE_y=data_trc.LKJC_Z*toMeters;   

RHIP_x=data_trc.RHJC_Y*toMeters;   RHIP_y=data_trc.RHJC_Z*toMeters;    
LHIP_x=data_trc.LHJC_Y*toMeters;   LHIP_y=data_trc.LHJC_Z*toMeters;  

PELO_x=data_trc.PELO_Y*toMeters;   PELO_y=data_trc.PELO_Z*toMeters;    
PELP_x=data_trc.PELP_Y*toMeters;   PELP_y=data_trc.PELP_Z*toMeters;   

TRXO_x=data_trc.TRXO_Y*toMeters;   TRXO_y=data_trc.TRXO_Z*toMeters;    
TRXP_x=data_trc.TRXP_Y*toMeters;   TRXP_y=data_trc.TRXP_Z*toMeters;  

FP1_force_x=data_grf_s.FP1_Force_Y;            FP1_force_y=data_grf_s.FP1_Force_Z;
FP1_COP_x  =data_grf_s.FP1_COP_Y*toMeters;     FP1_COP_y  =data_grf_s.FP1_COP_Z*toMeters;

FP2_force_x=data_grf_s.FP2_Force_Y;            FP2_force_y=data_grf_s.FP2_Force_Z;
FP2_COP_x=data_grf_s.FP2_COP_Y*toMeters;       FP2_COP_y=data_grf_s.FP2_COP_Z*toMeters;


%% Here begins our code
n = 2; % We add two extra frames to the gait cycle for derivations later

rightTimeRange = (237:336+n);
leftTimeRange = (288:386+n);

% Calculate the Trunk Angle
trunkAngleR = getSegmentAngle(TRXO_x, TRXO_y, TRXP_x, TRXP_y, rightTimeRange);
trunkAngleL = getSegmentAngle(TRXO_x, TRXO_y, TRXP_x, TRXP_y, leftTimeRange);

% Calculate the Pelvis Angle
pelvisAngleR = getSegmentAngle(PELP_x, PELP_y, PELO_x, PELO_y, rightTimeRange);
pelvisAngleL = getSegmentAngle(PELP_x, PELP_y, PELO_x, PELO_y, leftTimeRange);

leftThighAngleL = getSegmentAngle(LHIP_x, LHIP_y, LKNEE_x, LKNEE_y, leftTimeRange);
leftHipAngleL = getJointAngle(pelvisAngleL, leftThighAngleL);

% Calculate the right Hip Angle
rightThighAngleR = getSegmentAngle(RHIP_x, RHIP_y, RKNEE_x, RKNEE_y, rightTimeRange);
rightHipAngleR = getJointAngle(pelvisAngleR, rightThighAngleR);

% Calculate the left Knee Angle
leftShankAngleL = getSegmentAngle(LKNEE_x, LKNEE_y, LANKLE_x, LANKLE_y, leftTimeRange);
leftKneeAngleL = getJointAngle(leftShankAngleL, leftThighAngleL);

% Calculate the right Knee Angle
rightShankAngleR = getSegmentAngle(RKNEE_x, RKNEE_y, RANKLE_x, RANKLE_y, rightTimeRange);
rightKneeAngleR = getJointAngle(rightShankAngleR, rightThighAngleR);

% Calculate the right Ankle Angle
rightFootAngleR = getFootAngle(RANKLE_x, RANKLE_y, RTOE_x, RTOE_y, rightTimeRange);
rightAnkleAngleR = getAnkleAngle(rightShankAngleR, rightFootAngleR);

% Calculate the left Ankle Angle
leftFootAngleL = getFootAngle(LANKLE_x, LANKLE_y, LTOE_x, LTOE_y, leftTimeRange);
leftAnkleAngleL = getAnkleAngle(leftShankAngleL, leftFootAngleL);

% Save the angles to a separate file
left_table = table(trunkAngleL, pelvisAngleL, leftHipAngleL, leftThighAngleL, leftShankAngleL, leftKneeAngleL, leftFootAngleL, leftAnkleAngleL);
writetable(left_table,'angles_left.txt', 'Delimiter',' ')

right_table = table(trunkAngleR, pelvisAngleR, rightHipAngleR, rightThighAngleR, rightShankAngleR, rightKneeAngleR, rightFootAngleR, rightAnkleAngleR);
writetable(right_table,'angles_right.txt', 'Delimiter',' ')

%% Here begins the plots - Top->Down 
% Get the frames for the gait cycle
rightTimeRange_plot = (237:336);
leftTimeRange_plot = (288:386);

% Set the time to one gait cycle
timeR = linspace(0, 100, length(rightTimeRange_plot));
timeL = linspace(0, 100, length(leftTimeRange_plot));

% Plot all the angles on the same figure
% tiledlayout(4,2)

% Trunk Angle Plot
subplot(3,2,1);
plot(timeR, trunkAngleR(n+1:end), 'green', 'LineWidth', 1.5);
hold on
plot(timeL, trunkAngleL(n+1:end), 'red', 'LineWidth', 1.5);
title('Trunk')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Posterior tilt - / Anterior tilt + [deg]', 'FontSize', 9)
grid on

% Pelvis Angle Plot
subplot(3,2,2);
plot(timeR, pelvisAngleR(n+1:end), 'green', 'LineWidth', 1.5);
hold on
plot(timeL, pelvisAngleL(n+1:end), 'red', 'LineWidth', 1.5);
title('Pelvis')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Posterior tilt - / Anterior tilt + [deg]', 'FontSize', 9)
axis([0 100 0 8])
grid on

% Hip Angles Plot
subplot(3,2,3);
plot(timeR, rightHipAngleR(n+1:end), 'green', 'LineWidth', 1.5);
hold on
plot(timeL, leftHipAngleL(n+1:end), 'red', 'LineWidth', 1.5);
title('Hips')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Extension - / Flexion + [deg]', 'FontSize', 9)
axis([0 100 -30 40])
grid on

% Knee Angle Plots
subplot(3,2,4);
plot(timeR, rightKneeAngleR(n+1:end), 'green', 'LineWidth', 1.5);
hold on
plot(timeL, leftKneeAngleL(n+1:end), 'red', 'LineWidth', 1.5);
title('Knees')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Hyperextension - / Flexion + [deg]', 'FontSize', 9)
grid on

% Ankle Angle Plots
subplot(3,2,5);
plot(timeR, rightAnkleAngleR(n+1:end), 'green', 'LineWidth', 1.5);
hold on
plot(timeL, leftAnkleAngleL(n+1:end), 'red', 'LineWidth', 1.5);
title('Ankles')
legend('Right gait', 'Left gait')
xlabel('Gait cycle [%]')
ylabel('Plantarflexor - / Dorsiflexor + [deg]', 'FontSize', 9)
axis([0 100 -25 20])
grid on

%% Functions that handle angles for our code

% Function that calculates the angle of a segment
% This function does not work for the foot
function segmentAngle = getSegmentAngle(first_x, first_y, second_x, second_y, timeRange)
    segmentAngle = atand((first_x(timeRange)-second_x(timeRange))./(first_y(timeRange)-second_y(timeRange)));
end

% Function that calculates the Foot Angle
function footAngle = getFootAngle(ankle_x, ankle_y, toe_x, toe_y, timeRange)
    footAngle = atand((toe_y(timeRange)-ankle_y(timeRange))./(toe_x(timeRange)-ankle_x(timeRange)));
end

% Function that calculates the angle of a joint
% This funktion does not work for the ankle
function jointAngle = getJointAngle(upperSegment, lowerSegment)
    jointAngle = upperSegment - lowerSegment;
end

% Function that calculates the Ankle Angle
function ankleAngle = getAnkleAngle(shankAngle, footAngle)
    ankleAngle = shankAngle + footAngle;
    % 5 degree offset due to limitations in points
    ankleAngle = ankleAngle + 5;
end
