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

rightTimeRange = (237:336);
leftTimeRange = (288:386);

% Calculate the Trunk Angle
trunkAngleR = getSegmentAngle(TRXO_x, TRXO_y, TRXP_x, TRXP_y, rightTimeRange);
trunkAngleL = getSegmentAngle(TRXO_x, TRXO_y, TRXP_x, TRXP_y, leftTimeRange);

% Calculate the Pelvis Angle
pelvisAngleR = getSegmentAngle(PELP_x, PELP_y, PELO_x, PELO_y, rightTimeRange);
pelvisAngleL = getSegmentAngle(PELP_x, PELP_y, PELO_x, PELO_y, leftTimeRange);

% Calculate the left Hip Angle
leftThighAngleR = getSegmentAngle(LHIP_x, LHIP_y, LKNEE_x, LKNEE_y, rightTimeRange);
leftHipAngleR = getJointAngle(pelvisAngleR, leftThighAngleR);

leftThighAngleL = getSegmentAngle(LHIP_x, LHIP_y, LKNEE_x, LKNEE_y, leftTimeRange);
leftHipAngleL = getJointAngle(pelvisAngleL, leftThighAngleL);

% Calculate the right Hip Angle
rightThighAngleR = getSegmentAngle(RHIP_x, RHIP_y, RKNEE_x, RKNEE_y, rightTimeRange);
rightHipAngleR = getJointAngle(pelvisAngleR, rightThighAngleR);

rightThighAngleL = getSegmentAngle(RHIP_x, RHIP_y, RKNEE_x, RKNEE_y, leftTimeRange);
rightHipAngleL = getJointAngle(pelvisAngleL, rightThighAngleL);

% Calculate the left Knee Angle
leftShankAngleR = getSegmentAngle(LKNEE_x, LKNEE_y, LANKLE_x, LANKLE_y, rightTimeRange);
leftKneeAngleR = getJointAngle(leftThighAngleR, leftShankAngleR);

leftShankAngleL = getSegmentAngle(LKNEE_x, LKNEE_y, LANKLE_x, LANKLE_y, leftTimeRange);
leftKneeAngleL = getJointAngle(leftThighAngleL, leftShankAngleL);

% Calculate the right Knee Angle
rightShankAngleR = getSegmentAngle(RKNEE_x, RKNEE_y, RANKLE_x, RANKLE_y, rightTimeRange);
rightKneeAngleR = getJointAngle(rightThighAngleR, rightShankAngleR);

rightShankAngleL = getSegmentAngle(RKNEE_x, RKNEE_y, RANKLE_x, RANKLE_y, leftTimeRange);
rightKneeAngleL = getJointAngle(rightThighAngleL, rightShankAngleL);

% Calculate the right Ankle Angle
rightFootAngleR = getFootAngle(RANKLE_x, RANKLE_y, RTOE_x, RTOE_y, rightTimeRange);
rightAnkleAngleR = getAnkleAngle(rightShankAngleR, rightFootAngleR);

rightFootAngleL = getFootAngle(RANKLE_x, RANKLE_y, RTOE_x, RTOE_y, leftTimeRange);
rightAnkleAngleL = getAnkleAngle(rightShankAngleL, rightFootAngleL);


% Calculate the left Ankle Angle
leftFootAngleR = getFootAngle(LANKLE_x, LANKLE_y, LTOE_x, LTOE_y, rightTimeRange);
leftAnkleAngleR = getAnkleAngle(leftShankAngleR, leftFootAngleR);

leftFootAngleL = getFootAngle(LANKLE_x, LANKLE_y, LTOE_x, LTOE_y, leftTimeRange);
leftAnkleAngleL = getAnkleAngle(leftShankAngleL, leftFootAngleL);


%% Here begins the plots - Top->Down

% Set the time to one gait cycle
timeR = linspace(0, 100, length(rightTimeRange));
timeL = linspace(0, 100, length(leftTimeRange));

% Plot all the angles
tiledlayout(4,2)

% Trunk Angle Plot
nexttile
plot(timeR, trunkAngleR, 'green');
hold on
plot(timeL, trunkAngleL, 'red');
title('Trunk, right gait cycle green and left gait cycle red')
xlabel('Gait cycle [%]')
ylabel('Trunk Angle [deg]')

% Pelvis Angle Plot
nexttile
plot(timeR, pelvisAngleR, 'green');
hold on
plot(timeL, pelvisAngleL, 'red');
title('Pelvis right gait cycle green and left gait cycle red')
xlabel('Gait cycle [%]')
ylabel('Pelvis Angle [deg]')

% Left and Right Hip Angles Plot
nexttile
plot(timeR, leftHipAngleR, 'green');
hold on
plot(timeL, rightHipAngleL, 'red');
title('Left hip in right gait cycle (green) and right hip in left gait cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Hip Angle [deg]')

nexttile
plot(timeL, leftHipAngleL, 'green');
hold on
plot(timeR, rightHipAngleR, 'red');
title('Left hip in left gait cycle (green) and right hip in right gait cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Hip Angle [deg]')

% Knee Angle Plots
nexttile
plot(timeR, leftKneeAngleR, 'green');
hold on
plot(timeL, rightKneeAngleL, 'red');
title('Left knee in right gait cycle (green) and right knee in left gait cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Knee Angle [deg]')

nexttile
plot(timeL, leftKneeAngleL, 'green');
hold on
plot(timeR, rightKneeAngleR, 'red');
title('Left knee in left gait cycle (green) and right knee in right gait cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Knee Angle [deg]')

% Ankle Angle Plots
nexttile
plot(timeR, leftAnkleAngleR, 'green');
hold on
plot(timeL, rightAnkleAngleL, 'red');
title('Left ankle in right gait cycle (green) and right ankle in left gait cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Ankle Angle [deg]')

nexttile
plot(timeL, leftAnkleAngleL, 'green');
hold on
plot(timeR, rightAnkleAngleR, 'red');
title('Left ankle in left gait cycle (green) and right ankle in right gair cycle (red)')
xlabel('Gait cycle [%]')
ylabel('Ankle Angle [deg]')


%% Functions for our code

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
