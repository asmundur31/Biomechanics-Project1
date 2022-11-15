% Plot walking vs jogging --> right_gait
n=2;

% Load angle data
angles_jogging = readtable(fullfile(file_dir, 'jogging_angles_right.txt'));
angles_walking = readtable(fullfile(file_dir, 'walking_angles_right.txt'));

% Get the frames for the gait cycle
jogTimeRange = (163:229);
walkTimeRange = (237:336);

% Set the time to one gait cycle
timeJog = linspace(0, 100, length(jogTimeRange));
timeWalk = linspace(0, 100, length(walkTimeRange));

% Plot all the angles on the same figure
% Trunk Angle Plot
subplot(3,2,1);
plot(timeJog, angles_jogging.trunkAngleR(n+1:end), 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, angles_walking.trunkAngleR(n+1:end), 'cyan', 'LineWidth', 1.5);
title('Trunk')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Posterior tilt - / Anterior tilt + [deg]', 'FontSize', 9)
grid on

% Pelvis Angle Plot
subplot(3,2,2);
plot(timeJog, angles_jogging.pelvisAngleR(n+1:end), 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, angles_walking.pelvisAngleR(n+1:end), 'cyan', 'LineWidth', 1.5);
title('Pelvis')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Posterior tilt - / Anterior tilt + [deg]', 'FontSize', 9)
% axis([0 100 0 8])
grid on

% Hip Angles Plot
subplot(3,2,3);
plot(timeJog, angles_jogging.rightHipAngleR(n+1:end), 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, angles_walking.rightHipAngleR(n+1:end), 'cyan', 'LineWidth', 1.5);
title('Hips')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Extension - / Flexion + [deg]', 'FontSize', 9)
%axis([0 100 -30 40])
grid on

% Knee Angle Plots
subplot(3,2,4);
plot(timeJog, angles_jogging.rightKneeAngleR(n+1:end), 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, angles_walking.rightKneeAngleR(n+1:end), 'cyan', 'LineWidth', 1.5);
title('Knees')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Hyperextension - / Flexion + [deg]', 'FontSize', 9)
grid on

% Ankle Angle Plots
subplot(3,2,5);
plot(timeJog, angles_jogging.rightAnkleAngleR(n+1:end), 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, angles_walking.rightAnkleAngleR(n+1:end), 'cyan', 'LineWidth', 1.5);
title('Ankles')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Plantarflexor - / Dorsiflexor + [deg]', 'FontSize', 9)
%axis([0 100 -25 20])
grid on



