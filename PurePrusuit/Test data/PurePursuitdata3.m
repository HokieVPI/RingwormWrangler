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
waypoints5 = [631 329;
              544 549;
              499 706;
              597 862;
              740 1019];

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

%% test_3_23_8
waypoints8 = [890 308;
              890 1097];

fid = fopen("test_3_27_1", "r");
lines = {};
while ~feof(fid)
    lines{end+1} = fgetl(fid);
end
fclose(fid);

goalX = []; goalY = [];
desiredHeading = []; globalAzimuth = [];
currentX = []; currentY = [];
pathCompleteIdx = NaN;

idx = 2;
while idx + 5 <= length(lines)
    val1 = str2double(lines{idx});
    if isnan(val1)
        pathCompleteIdx = length(currentX);
        break;
    end
    goalX          = [goalX;          val1];
    goalY          = [goalY;          str2double(lines{idx+1})];
    desiredHeading = [desiredHeading; str2double(lines{idx+2})];
    globalAzimuth  = [globalAzimuth;  str2double(lines{idx+3})];
    currentX       = [currentX;       str2double(lines{idx+4})];
    currentY       = [currentY;       str2double(lines{idx+5})];
    idx = idx + 6;
end

valid = currentY >= 0;
currentX = currentX(valid); currentY = currentY(valid);
desiredHeading = desiredHeading(valid); globalAzimuth = globalAzimuth(valid);
goalX = goalX(valid); goalY = goalY(valid);

if ~isnan(pathCompleteIdx)
    cumValid = cumsum(valid);
    pcValid = find(cumValid == pathCompleteIdx, 1, 'first');
    if isempty(pcValid)
        pcValid = length(currentX);
    end
else
    pcValid = length(currentX);
end

figure
hold on
scatter(currentX, currentY, 'filled')
scatter(waypoints8(:,1), waypoints8(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))], 'HandleVisibility', 'off')
end
plot(currentX(pcValid), currentY(pcValid), 'r*', 'MarkerSize', 15, 'LineWidth', 2)
title("Pure Pursuit X-Y Position - Test 8")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points", "Path Complete")
axis equal
hold off

%% test_3_24_2
waypoints9 = [75 60;
              145 75;
              250 90];

fid = fopen("test_3_24_2", "r");
if fid == -1
    warning("test_3_24_2 not found, skipping section.");
else
lines = {};
while ~feof(fid)
    lines{end+1} = fgetl(fid);
end
fclose(fid);

goalX = []; goalY = [];
desiredHeading = []; globalAzimuth = [];
currentX = []; currentY = [];
pathCompleteIdx = NaN;

idx = 2;
while idx + 5 <= length(lines)
    val1 = str2double(lines{idx});
    if isnan(val1)
        pathCompleteIdx = length(currentX);
        break;
    end
    goalX          = [goalX;          val1];
    goalY          = [goalY;          str2double(lines{idx+1})];
    desiredHeading = [desiredHeading; str2double(lines{idx+2})];
    globalAzimuth  = [globalAzimuth;  str2double(lines{idx+3})];
    currentX       = [currentX;       str2double(lines{idx+4})];
    currentY       = [currentY;       str2double(lines{idx+5})];
    idx = idx + 6;
end

valid = currentY >= 0;
currentX = currentX(valid); currentY = currentY(valid);
desiredHeading = desiredHeading(valid); globalAzimuth = globalAzimuth(valid);
goalX = goalX(valid); goalY = goalY(valid);

if ~isnan(pathCompleteIdx)
    cumValid = cumsum(valid);
    pcValid = find(cumValid == pathCompleteIdx, 1, 'first');
    if isempty(pcValid)
        pcValid = length(currentX);
    end
else
    pcValid = length(currentX);
end

figure
hold on
scatter(currentX, currentY, 'filled')
scatter(waypoints9(:,1), waypoints9(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))], 'HandleVisibility', 'off')
end
plot(currentX(pcValid), currentY(pcValid), 'r*', 'MarkerSize', 15, 'LineWidth', 2)
title("Pure Pursuit X-Y Position - Test 3\_24\_2")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points", "Path Complete")
axis equal
hold off
end

%% test_3_24_3
waypoints_3_24_3 = [259 183;
                    360 311];

fid = fopen("test_3_24_3", "r");
if fid == -1
    warning("test_3_24_3 not found, skipping section.");
else
lines = {};
while ~feof(fid)
    lines{end+1} = fgetl(fid);
end
fclose(fid);

goalX = []; goalY = [];
desiredHeading = []; globalAzimuth = [];
currentX = []; currentY = [];
pathCompleteIdx = NaN;

idx = 2;
while idx + 5 <= length(lines)
    val1 = str2double(lines{idx});
    if isnan(val1)
        pathCompleteIdx = length(currentX);
        break;
    end
    goalX          = [goalX;          val1];
    goalY          = [goalY;          str2double(lines{idx+1})];
    desiredHeading = [desiredHeading; str2double(lines{idx+2})];
    globalAzimuth  = [globalAzimuth;  str2double(lines{idx+3})];
    currentX       = [currentX;       str2double(lines{idx+4})];
    currentY       = [currentY;       str2double(lines{idx+5})];
    idx = idx + 6;
end

valid = currentY >= 0;
currentX = currentX(valid); currentY = currentY(valid);
desiredHeading = desiredHeading(valid); globalAzimuth = globalAzimuth(valid);
goalX = goalX(valid); goalY = goalY(valid);

if ~isnan(pathCompleteIdx)
    cumValid = cumsum(valid);
    pcValid = find(cumValid == pathCompleteIdx, 1, 'first');
    if isempty(pcValid)
        pcValid = length(currentX);
    end
else
    pcValid = length(currentX);
end

figure
hold on
scatter(currentX, currentY, 'filled')
scatter(waypoints_3_24_3(:,1), waypoints_3_24_3(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))], 'HandleVisibility', 'off')
end
plot(currentX(pcValid), currentY(pcValid), 'r*', 'MarkerSize', 15, 'LineWidth', 2)
title("Pure Pursuit X-Y Position - Test 3\_24\_3")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points", "Path Complete")
axis equal
hold off
end
%% Test 3_29_1 
waypoints3_29_1 = [1300 500;
                   1300 800;
                   950 800;
                   950 400;
                   550 400;
                   550 800;
                   200 800;
                   200 400];

raw = importdata("test_3_31_6");
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
scatter(waypoints3_29_1(:,1), waypoints3_29_1(:,2), 100, 'm', 'filled')
scatter(goalX, goalY, 25, 'g', 'filled')
for k = 1:length(desiredHeading)
    plot([currentX(k), currentX(k)+4*cos(desiredHeading(k)*(pi/180))], ...
         [currentY(k), currentY(k)+4*sin(desiredHeading(k)*(pi/180))],'LineWidth',1)
end
title("Pure Pursuit X-Y Position - Test 3_29_1")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position", "Waypoints", "Goal Points")
axis equal
grid on 
hold off

