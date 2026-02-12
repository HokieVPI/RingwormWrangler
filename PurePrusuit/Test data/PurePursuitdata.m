clc
clear
close all

% data 1
data1 = importdata("test_2_10_26");
amounts = data1.data;
robotx = [];
roboty = [];
goalx = [];
goaly = [];
i = 1;
while i <= length(amounts)
    goalx = [goalx; amounts(i)];
    goaly = [goaly; amounts(i+1)];
    robotx = [robotx; amounts(i+2)];
    roboty = [roboty; amounts(i+3)];
    i = i+4;
end
robotx = robotx;
roboty = roboty;
goalx = goalx;
goaly = goaly;
figure
hold on
line(robotx,roboty)
scatter(goalx,goaly,'Color','r')
xlim([0, 2133.6])
ylim([0, 2641])
hold off


% data 2
data2 = readtable("test2_2_10_26");
robotx = [];
desiredHeading = [];
globalHeading = [];
roboty = [];
goalx = [];
goaly = [];
curvatureCoeff = [];
i = 1;
while i <= height(data2)-7
    desiredHeading = [desiredHeading; data2(i,2)];
    globalHeading = [globalHeading; data2(i+1,2)];
    robotx = [robotx; data2(i+2,1)];
    roboty = [roboty; data2(i+3,1)];
    curvatureCoeff = [curvatureCoeff; data2(i+4,2)];
    goalx = [goalx; data2(i+5,1)];
    goaly = [goaly; data2(i+6,1)];
    i = i+7;
end
desiredHeading = table2array(desiredHeading);
globalHeading = table2array(globalHeading);
robotx = table2array(robotx);
roboty = table2array(roboty);
curvatureCoeff = table2array(curvatureCoeff);
goalx = table2array(goalx);
goaly = table2array(goaly);
figure
hold on
line(robotx,roboty)
scatter(goalx,goaly,'Color','r')
xlim([0, 2133.6])
ylim([0, 2641])
hold off



% data 3

data3 = readtable("test3_2_10_26");
robotx = [];
desiredHeading = [];
globalHeading = [];
roboty = [];
goalx = [];
goaly = [];
curvatureCoeff = [];
i = 1;
while i <= height(data3)-7
    desiredHeading = [desiredHeading; data3(i,2)];
    globalHeading = [globalHeading; data3(i+1,2)];
    robotx = [robotx; data3(i+2,1)];
    roboty = [roboty; data3(i+3,1)];
    curvatureCoeff = [curvatureCoeff; data3(i+4,2)];
    goalx = [goalx; data3(i+5,1)];
    goaly = [goaly; data3(i+6,1)];
    i = i+7;
end
desiredHeading = table2array(desiredHeading);
globalHeading = table2array(globalHeading);
robotx = table2array(robotx);
roboty = table2array(roboty);
curvatureCoeff = table2array(curvatureCoeff);
goalx = table2array(goalx);
goaly = table2array(goaly);
figure
hold on
line(robotx,roboty)
scatter(goalx,goaly,'Color','r')
xlim([0, 2133.6])
ylim([0, 2641])
hold off

% data 4

data4 = readtable("test4_2_10_26");
robotx = [];
desiredHeading = [];
globalHeading = [];
roboty = [];
goalx = [];
goaly = [];
curvatureCoeff = [];
idk1 = [];
idk2 = [];
i = 1;
while i <= height(data4)-7
    desiredHeading = [desiredHeading; data4(i,2)];
    globalHeading = [globalHeading; data4(i+1,2)];
    robotx = [robotx; data4(i+2,1)];
    roboty = [roboty; data4(i+3,1)];
    idk1 = [idk1; data4(i+4,1)];
    idk2 = [idk2; data4(i+5,1)];
    curvatureCoeff = [curvatureCoeff; data4(i+6,2)];
    goalx = [goalx; data4(i+7,1)];
    goaly = [goaly; data4(i+8,1)];
    i = i+9;
end
desiredHeading = table2array(desiredHeading)
globalHeading = table2array(globalHeading)
robotx = table2array(robotx)
roboty = table2array(roboty)
curvatureCoeff = table2array(curvatureCoeff)
goalx = table2array(goalx)
goaly = table2array(goaly)
figure
hold on
line(robotx,roboty)
scatter(goalx,goaly,'Color','r')
xlim([0, 2133.6])
ylim([0, 2641])
grid on 
hold off


% %% data 5
% 
% data5 = readtable("test5_2_10_26");
% robotx = [];
% desiredHeading = [];
% globalHeading = [];
% roboty = [];
% goalx = [];
% goaly = [];
% curvatureCoeff = [];
% idk1 = [];
% idk2 = [];
% i = 1;
% while i <= height(data5)-7
%     desiredHeading = [desiredHeading; data5(i,2)];
%     globalHeading = [globalHeading; data5(i+1,2)];
%     robotx = [robotx; data5(i+2,1)];
%     roboty = [roboty; data5(i+3,1)];
%     idk1 = [idk1; data5(i+4,1)];
%     idk2 = [idk2; data5(i+5,1)];
%     curvatureCoeff = [curvatureCoeff; data5(i+6,2)];
%     goalx = [goalx; data5(i+7,1)];
%     goaly = [goaly; data5(i+8,1)];
%     i = i+9;
% end
% % desiredHeading = table2array(desiredHeading)
% % globalHeading = table2array(globalHeading)
% robotx = table2array(robotx)
% roboty = table2array(roboty)
% % curvatureCoeff = table2array(curvatureCoeff)
% goalx = table2array(goalx)
% goaly = table2array(goaly)
% figure
% hold on
% line(robotx,roboty)
% scatter(goalx,goaly,'Color','r')
% xlim([0, 2133.6])
% ylim([0, 2641])
% grid on
% hold off


