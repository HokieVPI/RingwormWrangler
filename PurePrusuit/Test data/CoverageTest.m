clc; clear; close all; 
%% test_3_31_night12
% Waypoints match RW_Tag_PF_v2.ino path[] (PATH_LENGTH = 14), units cm
path_cm = [
    315.0,  415.0;
    315.0, 2405.5;
    427.9, 2405.5;
    427.9,  315.0;
    610.8,  315.0;
    610.8, 2405.5;
    793.6, 2405.5;
    793.6,  315.0;
    976.5,  315.0;
    976.5, 2405.5;
    1159.4, 2405.5;
    1159.4,  315.0;
    737.0,  315.0;
    315.0,  415.0
];

raw = importdata("test_4_7_2");
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