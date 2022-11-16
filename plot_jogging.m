% Plot walking vs jogging --> right_gait
n=2;
file_dir = pwd;

% Load angle data
angles_jogging = readtable(fullfile(file_dir, 'jogging_angles_right.txt'));
angles_walking = readtable(fullfile(file_dir, 'walking_angles_right.txt'));

kinetics_jogging = readtable(fullfile(file_dir, 'jogging_kinetics.txt'));
kinetics_walking = readtable(fullfile(file_dir, 'walking_kinetics.txt'));

% Get the frames for the gait cycle
jogTimeRange = (163:229);
walkTimeRange = (237:336);

% Set the time to one gait cycle
timeJog = linspace(0, 100, length(jogTimeRange));
timeWalk = linspace(0, 100, length(walkTimeRange));

%% ANGLES
% Plot all the angles on the same figure
% Trunk Angle Plot
figure(1)
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
axis([0 100 -30 40])
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
axis([0 100 -10 60])
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

%% KINETICS

% ANKLE
figure(2)
subplot(3, 1, 1); % Foot segment angular acceleration
plot(timeJog, kinetics_jogging.r_alfa_f, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_alfa_f, 'cyan', 'LineWidth', 1.5);
title('Foot segment angular acceleration')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -200 200])
grid on

subplot(3, 1, 2); % Ankle moment
plot(timeJog, -kinetics_jogging.r_moment_A, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, -kinetics_walking.r_moment_A, 'cyan', 'LineWidth', 1.5);
title('Ankle moment')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Dorsiflexor - / Plantarflexor + [N*m]', 'FontSize', 9)
% axis([0 100 -25 110])
grid on

subplot(3, 1, 3); % Ankle power
plot(timeJog, kinetics_jogging.r_power_A, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_power_A, 'cyan', 'LineWidth', 1.5);
title('Ankle power')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
grid on

% KNEE
figure(3)
subplot(3, 1, 1); % Shank segment angular acceleration
plot(timeJog, kinetics_jogging.r_alfa_S, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_alfa_S, 'cyan', 'LineWidth', 1.5);
title('Shank segment angular acceleration')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -200 200])
grid on

subplot(3, 1, 2); % Knee moment
plot(timeJog, kinetics_jogging.r_moment_K, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_moment_K, 'cyan', 'LineWidth', 1.5);
title('Knee moment')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Knee flexor - / Knee extensor + [N*m]', 'FontSize', 9)
axis([0 100 -50 80])
grid on

subplot(3, 1, 3); % Knee power
plot(timeJog, kinetics_jogging.r_power_K, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_power_K, 'cyan', 'LineWidth', 1.5);
title('Knee power')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
grid on

% HIP
figure(4)
subplot(3, 1, 1); % Thigh segment angular acceleration
plot(timeJog, kinetics_jogging.r_alfa_T, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_alfa_T, 'cyan', 'LineWidth', 1.5);
title('Thigh segment angular acceleration')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Angular acceleration [rad/s^2]', 'FontSize', 9)
axis([0 100 -100 80])
grid on

subplot(3, 1, 2); % Hip moment
plot(timeJog, kinetics_jogging.r_moment_H, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_moment_H, 'cyan', 'LineWidth', 1.5);
title('Hip moment')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Hip flexor - / Hip extensor + [N*m]', 'FontSize', 9)
%axis([0 100 -25 110])
grid on

subplot(3, 1, 3); % Hip power
plot(timeJog, kinetics_jogging.r_power_H, 'blue', 'LineWidth', 1.5);
hold on
plot(timeWalk, kinetics_walking.r_power_H, 'cyan', 'LineWidth', 1.5);
title('Hip power')
legend('Jogging', 'Walking')
xlabel('Gait cycle [%]')
ylabel('Power absorption - / Power generation + [W]', 'FontSize', 9)
axis([0 100 -6000 10000])
grid on


