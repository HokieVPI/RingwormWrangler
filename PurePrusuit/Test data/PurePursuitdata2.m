clc
clear
close all

%% incoming test
% what I need: 
% robot x and y
% goal x and y
% radius of curvature
% heading (angle from goal) and azimuth (angle in room)


%% Canada test1
mapArea = [0 0;
    70*2.54*12 0;
    70*2.54*12 32.4*2.54*12;
    (70-2)*2.54*12 (32.4)*2.54*12;
    (70-2)*2.54*12 (32.4+19.81)*2.54*12;
    (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12;
    (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12;
    (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12;
    (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12;
    0 (32.4+19.81+14.75+20)*2.54*12;
    0 0];

data5_1= importdata("test_3_23_1.asv");
data5_1 = data5_1.data;
robotx = [];
roboty = [];
Azimuth = [];
i = 1;
while i <= height(data5_1)-3
    robotx = [robotx; data5_1(i,1)];
    roboty = [roboty; data5_1(i+1,1)];
    Azimuth = [Azimuth; data5_1(i+2,1)];
    i = i+3;
end

figure
hold on
scatter(robotx,roboty)
for i = 1:length(Azimuth)
    plot([robotx(i), robotx(i)+4*cos(Azimuth(i)*(pi/180))],[roboty(i), roboty(i)+4*sin(Azimuth(i)*(pi/180))])
end
% xlim([0, 396])
% ylim([0, 244])
title("Reference Positioning Test Canada")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
axis equal
hold off


%% Canada 2

data5_2= importdata("test_3_23_2");
data5_2 = data5_2.data;
robotx = [];
roboty = [];
Azimuth = [];
i = 1;
while i <= height(data5_2)-3
    robotx = [robotx; data5_2(i,1)];
    roboty = [roboty; data5_2(i+1,1)];
    Azimuth = [Azimuth; data5_2(i+2,1)];
    i = i+3;
end

figure
hold on
scatter(robotx,roboty)
for i = 1:length(Azimuth)
    plot([robotx(i), robotx(i)+4*cos(Azimuth(i)*(pi/180))],[roboty(i), roboty(i)+4*sin(Azimuth(i)*(pi/180))])
end
% xlim([0, 396])
% ylim([0, 244])
title("Reference Positioning Test Canada")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
axis equal
hold off
