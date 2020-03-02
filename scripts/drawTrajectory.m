%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%                         drawTrajectory.m                        %%%%%
%%%%%                            Raúl Tapia                           %%%%%
%%%%%                          Robótica Móvil                         %%%%%
%%%%% Master Universitario en Robótica - Universidad Miguel Hernández %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% @file   drawTrajectory.m
% @brief  Plot visual odometry pose estimation.
% @author Raúl Tapia

clc; clear; close all;

%%% Read data log
visualOdometry = importdata("../../rosbag/visual_odometry_pose.txt");
groundTruth = importdata("../../rosbag/visual_odometry_ground_truth.txt");

%%% Plot position 3D
figure('Color', [1 1 1]);
plot3(visualOdometry(:,2), visualOdometry(:,3), visualOdometry(:,4), '-b');
hold on;
plot3(groundTruth(:,2), groundTruth(:,3), groundTruth(:,4), '--r');
grid on;
xlabel('x'); ylabel('y'); zlabel('z');
legend({'Visual odometry', 'Ground-truth'},'FontSize',14);
set(gca,'FontSize',14);

%%% Plot position 2D
figure('Color', [1 1 1]);

subplot(3,1,1);
plot(visualOdometry(:,1), visualOdometry(:,2), '-or'); hold on;
plot(groundTruth(:,1), groundTruth(:,2), '--r');
xlabel('t'); ylabel('x');
legend({'Visual odometry', 'Ground-truth'},'FontSize',14);
set(gca,'FontSize',14);

subplot(3,1,2);
plot(visualOdometry(:,1), visualOdometry(:,3), '-og'); hold on;
plot(groundTruth(:,1), groundTruth(:,3), '--g');
xlabel('t'); ylabel('y');
legend({'Visual odometry', 'Ground-truth'},'FontSize',14);
set(gca,'FontSize',14);

subplot(3,1,3);
plot(visualOdometry(:,1), visualOdometry(:,4), '-ob'); hold on;
plot(groundTruth(:,1), groundTruth(:,4), '--b');
xlabel('t'); ylabel('z');
legend({'Visual odometry', 'Ground-truth'},'FontSize',14);
set(gca,'FontSize',14);