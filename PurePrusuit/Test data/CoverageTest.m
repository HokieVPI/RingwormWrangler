clc; clear; close all; 
%% test_3_31_night12
% Waypoints match RW_Tag_PF_v2.ino path[] (PATH_LENGTH = 32), units cm
path_cm = [
    250.0,  500.0;
    250.0, 2400.5;
    333.0, 2400.5;
    416.0, 2400.5;
    416.0,  350.0;
    499.0,  250.0;
    582.0,  250.0;
    582.0, 2400.5;
    665.0, 2400.5;
    748.0, 2400.5;
    748.0,  250.0;
    831.0,  250.0;
    914.0,  250.0;
    914.0, 2400.5;
    997.0, 2400.5;
    1080.0, 2400.5;
    1080.0,  250.0;
    1163.0,  250.0;
    1246.0,  250.0;
    1246.0, 1790.9;
    1329.0, 1790.9;
    1412.0, 1790.9;
    1412.0,  250.0;
    1495.0,  250.0;
    1578.0,  250.0;
    1578.0, 1341.4;
    1661.0, 1341.4;
    1744.0, 1341.4;
    1744.0,  250.0;
    997.0,  250.0;
    250.0,  250.0;
    250.0,  500.0
];

raw = importdata("test_4_10_1");
raw = raw.data;

goalX = []; goalY = [];
desiredHeading = []; globalAzimuth = [];
currentX = []; currentY = [];

i = 1;
while i <= height(raw) - 5
    goalX          = [goalX;          raw(i,   1)];
    goalY          = [goalY;          raw(i+1, 1)];
    desiredHeading = [desiredHeading; raw(i+2, 1)];
    globalAzimuth  = [globalAzimuth;  raw(i+3, 1)];
    currentX       = [currentX;       raw(i+4, 1)];
    currentY       = [currentY;       raw(i+5, 1)];
    i = i + 6;
end

valid = currentY >= 0;
currentX = currentX(valid); currentY = currentY(valid);
desiredHeading = desiredHeading(valid); globalAzimuth = globalAzimuth(valid);
goalX = goalX(valid); goalY = goalY(valid);

figure
hold on
scatter(currentX, currentY, 'filled')
scatter(path_cm(:,1), path_cm(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+10*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+10*sin(desiredHeading(k)*(pi/180))],'LineWidth',1)
end
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
plot(mapArea(:,1),mapArea(:,2))
title("Pure Pursuit X-Y Position - test\_3\_31\_night12")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
grid on 
hold off