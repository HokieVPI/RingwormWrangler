clc
clear
close all

%% test_3_23_4
waypoints4 = [125 125;
              300 125;
              300 200];

raw = importdata("test_3_23_4");
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
scatter(waypoints4(:,1), waypoints4(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))])
end
title("Pure Pursuit X-Y Position - Test 4")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
hold off

%% test_3_23_5
waypoints5 = [125 125;
              300 125;
              300 200];

raw = importdata("test_3_23_5");
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
scatter(waypoints5(:,1), waypoints5(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))])
end
title("Pure Pursuit X-Y Position - Test 5")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
hold off

%% test_3_23_6
waypoints6 = [50 50;
              100 100;
              150 150];

raw = importdata("test_3_23_6");
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
scatter(waypoints6(:,1), waypoints6(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))])
end
title("Pure Pursuit X-Y Position - Test 6")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
hold off

%% test_3_23_7
waypoints7 = [75 75;
              87.5 100;
              100 125];

raw = importdata("test_3_23_7");
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
scatter(waypoints7(:,1), waypoints7(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))])
end
title("Pure Pursuit X-Y Position - Test 7")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
hold off
