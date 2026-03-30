
%% data3 postion test 1 

data3_6= importdata("test_3_29_2");
data3_6 = data3_6.data;
robotx = [];
roboty = [];
i = 1;
while i <= height(data3_6)-2
    robotx = [robotx; data3_6(i,1)];
    roboty = [roboty; data3_6(i+1,1)];
    i = i+2;
end
robotx = robotx;
roboty = roboty;
figure
hold on
scatter(robotx,roboty)
xlim([0, 2133.6])
ylim([0, 2641])
<<<<<<< Updated upstream
scatter(797.05,706.69,'magenta',"filled") % refrence point 
=======
scatter(631,392,'magenta',"filled") % refrence point 
>>>>>>> Stashed changes
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

title("Reference Positioning Test #2")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
% , "Actual Mat Position", "Mat Area")
hold off
% figure
% subplot(1,2,1)
% histogram(robotx)
% subplot(1,2,2)
% histogram(roboty)
% keep this

%% data3 postion test 2 

data3_6= importdata("test_3_28_3");
data3_6 = data3_6.data;
robotx = [];
roboty = [];
i = 1;
while i <= height(data3_6)-2
    robotx = [robotx; data3_6(i,1)];
    roboty = [roboty; data3_6(i+1,1)];
    i = i+2;
end
robotx = robotx;
roboty = roboty;
figure
hold on
scatter(robotx,roboty)
xlim([0, 2133.6])
ylim([0, 2641])
scatter(797.05,706.69,'magenta',"filled") % refrence point 
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

title("Reference Positioning Test #2")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
% , "Actual Mat Position", "Mat Area")
hold off
% figure
% subplot(1,2,1)
% histogram(robotx)
% subplot(1,2,2)
% histogram(roboty)
% keep this

%% data3 postion test 3
% bad rate of data 
data3_6= importdata("test_3_28_4");
data3_6 = data3_6.data;
robotx = [];
roboty = [];
i = 1;
while i <= height(data3_6)-2
    robotx = [robotx; data3_6(i,1)];
    roboty = [roboty; data3_6(i+1,1)];
    i = i+2;
end
robotx = robotx;
roboty = roboty;
figure
hold on
scatter(robotx,roboty)
xlim([0, 2133.6])
ylim([0, 2641])
scatter(768.71,1193.30,'magenta',"filled") % refrence point 
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

title("Reference Positioning Test #2")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
% , "Actual Mat Position", "Mat Area")
hold off
% figure
% subplot(1,2,1)
% histogram(robotx)
% subplot(1,2,2)
% histogram(roboty)
% keep this


%% data3 postion test 4

data3_6= importdata("test_3_28_7");
data3_6 = data3_6.data;
robotx = [];
roboty = [];
i = 1;
while i <= height(data3_6)-2
    robotx = [robotx; data3_6(i,1)];
    roboty = [roboty; data3_6(i+1,1)];
    i = i+2;
end
robotx = robotx;
roboty = roboty;
figure
hold on
scatter(robotx,roboty)
xlim([0, 2133.6])
ylim([0, 2641])
scatter(1320.70,711.40,'magenta',"filled") % refrence point 
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

title("Reference Positioning Test #2")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
% , "Actual Mat Position", "Mat Area")
hold off
% figure
% subplot(1,2,1)
% histogram(robotx)
% subplot(1,2,2)
% histogram(roboty)
% keep this


%% data3 postion test 5 (nlos) — uncomment; log = x y nlos1 nlos2 nlos3 NoSight per row
% here = fileparts(mfilename('fullpath'));
% [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fullfile(here, "LOG_5.txt"));
% figure
% hold on
% scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
% xlim([0, 2133.6])
% ylim([0, 2641])
% scatter(refX, refY, 'magenta', "filled") % reference point — set refX, refY
% mapArea = [0 0; 70*2.54*12 0; 70*2.54*12 32.4*2.54*12; (70-2)*2.54*12 (32.4)*2.54*12; (70-2)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12; 0 (32.4+19.81+14.75+20)*2.54*12; 0 0];
% plot(mapArea(:,1), mapArea(:,2))
% title("Reference Positioning Test #2")
% xlabel("x position (cm)")
% ylabel("y position (cm)")
% legend("Robot (red if nlos==0 on 2+ anchors)", "Reference", "Mat Area")
% hold off

%% data3 postion test 6 (nlos) — uncomment; log = x y nlos1 nlos2 nlos3 NoSight per row
% here = fileparts(mfilename('fullpath'));
% [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fullfile(here, "LOG_6.txt"));
% figure
% hold on
% scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
% xlim([0, 2133.6])
% ylim([0, 2641])
% scatter(refX, refY, 'magenta', "filled") % reference point — set refX, refY
% mapArea = [0 0; 70*2.54*12 0; 70*2.54*12 32.4*2.54*12; (70-2)*2.54*12 (32.4)*2.54*12; (70-2)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12; 0 (32.4+19.81+14.75+20)*2.54*12; 0 0];
% plot(mapArea(:,1), mapArea(:,2))
% title("Reference Positioning Test #2")
% xlabel("x position (cm)")
% ylabel("y position (cm)")
% legend("Robot (red if nlos==0 on 2+ anchors)", "Reference", "Mat Area")
% hold off

%% data3 postion test 7 (nlos) — uncomment; log = x y nlos1 nlos2 nlos3 NoSight per row
% here = fileparts(mfilename('fullpath'));
% [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fullfile(here, "LOG_7.txt"));
% figure
% hold on
% scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
% xlim([0, 2133.6])
% ylim([0, 2641])
% scatter(refX, refY, 'magenta', "filled") % reference point — set refX, refY
% mapArea = [0 0; 70*2.54*12 0; 70*2.54*12 32.4*2.54*12; (70-2)*2.54*12 (32.4)*2.54*12; (70-2)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12; 0 (32.4+19.81+14.75+20)*2.54*12; 0 0];
% plot(mapArea(:,1), mapArea(:,2))
% title("Reference Positioning Test #2")
% xlabel("x position (cm)")
% ylabel("y position (cm)")
% legend("Robot (red if nlos==0 on 2+ anchors)", "Reference", "Mat Area")
% hold off

%% data3 postion test 8 (nlos) — uncomment; log = x y nlos1 nlos2 nlos3 NoSight per row
% here = fileparts(mfilename('fullpath'));
% [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fullfile(here, "LOG_8.txt"));
% figure
% hold on
% scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
% xlim([0, 2133.6])
% ylim([0, 2641])
% scatter(refX, refY, 'magenta', "filled") % reference point — set refX, refY
% mapArea = [0 0; 70*2.54*12 0; 70*2.54*12 32.4*2.54*12; (70-2)*2.54*12 (32.4)*2.54*12; (70-2)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12; 0 (32.4+19.81+14.75+20)*2.54*12; 0 0];
% plot(mapArea(:,1), mapArea(:,2))
% title("Reference Positioning Test #2")
% xlabel("x position (cm)")
% ylabel("y position (cm)")
% legend("Robot (red if nlos==0 on 2+ anchors)", "Reference", "Mat Area")
% hold off

%% data3 postion test 9 (nlos) — uncomment; log = x y nlos1 nlos2 nlos3 NoSight per row
% here = fileparts(mfilename('fullpath'));
% [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fullfile(here, "LOG_9.txt"));
% figure
% hold on
% scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
% xlim([0, 2133.6])
% ylim([0, 2641])
% scatter(refX, refY, 'magenta', "filled") % reference point — set refX, refY
% mapArea = [0 0; 70*2.54*12 0; 70*2.54*12 32.4*2.54*12; (70-2)*2.54*12 (32.4)*2.54*12; (70-2)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81)*2.54*12; (70-2-12.5)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75)*2.54*12; (70-2-12.5-4.45)*2.54*12 (32.4+19.81+14.75+20)*2.54*12; 0 (32.4+19.81+14.75+20)*2.54*12; 0 0];
% plot(mapArea(:,1), mapArea(:,2))
% title("Reference Positioning Test #2")
% xlabel("x position (cm)")
% ylabel("y position (cm)")
% legend("Robot (red if nlos==0 on 2+ anchors)", "Reference", "Mat Area")
% hold off

% ----- local functions (for nlos sections above) -----

function [robotx, roboty, nlos1, nlos2, nlos3] = parseStaticUWBFile(fname)
    robotx = [];
    roboty = [];
    nlos1 = [];
    nlos2 = [];
    nlos3 = [];
    if exist(fname, 'file') ~= 2
        warning('StaticUWBtest:MissingFile', 'Not found: %s', fname);
        return
    end
    S = importdata(fname);
    if isstruct(S) && isfield(S, 'data')
        M = S.data;
    elseif isnumeric(S)
        M = S;
    else
        return
    end
    if isempty(M)
        return
    end
    [nR, nC] = size(M);
    if nC >= 5
        robotx = M(:, 1);
        roboty = M(:, 2);
        nlos1 = M(:, 3);
        nlos2 = M(:, 4);
        nlos3 = M(:, 5);
        return
    end
    if nC >= 2
        robotx = M(:, 1);
        roboty = M(:, 2);
        nlos1 = nan(nR, 1);
        nlos2 = nan(nR, 1);
        nlos3 = nan(nR, 1);
        return
    end
    if nC == 1 && nR >= 2
        robotx = M(1:2:end, 1);
        roboty = M(2:2:end, 1);
        nPts = numel(robotx);
        nlos1 = nan(nPts, 1);
        nlos2 = nan(nPts, 1);
        nlos3 = nan(nPts, 1);
    end
end

function scatterStaticUWB(robotx, roboty, nlos1, nlos2, nlos3)
    n = numel(robotx);
    if n == 0
        return
    end
    nlos1 = nlos1(:);
    nlos2 = nlos2(:);
    nlos3 = nlos3(:);
    if numel(nlos1) < n
        nlos1(end+1:n, 1) = nan;
    end
    if numel(nlos2) < n
        nlos2(end+1:n, 1) = nan;
    end
    if numel(nlos3) < n
        nlos3(end+1:n, 1) = nan;
    end
    hasNlos = ~isnan(nlos1) & ~isnan(nlos2) & ~isnan(nlos3);
    nZero = (nlos1 == 0) + (nlos2 == 0) + (nlos3 == 0);
    redMask = hasNlos & (nZero >= 2);
    scatter(robotx(~redMask), roboty(~redMask), 36, 'b', 'filled')
    hold on
    scatter(robotx(redMask), roboty(redMask), 36, 'r', 'filled')
end
