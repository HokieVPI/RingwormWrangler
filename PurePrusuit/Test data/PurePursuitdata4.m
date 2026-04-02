clc
clear
close all


data6_j= importdata("test_3_24_3");
data6_j = data6_j.data;
goalx = [];
goaly = [];
heading = [];
azimuth = [];
robotx = [];
roboty = [];
% portmotor =[];
% starboardmotor = [];
i = 1;
while i <= height(data6_j)-6
    goalx = [goalx; data6_j(i,1)];
    goaly = [goaly; data6_j(i+1,1)];
    heading = [heading; data6_j(i+2,1)];
    azimuth = [azimuth; data6_j(i+3,1)];
    robotx = [robotx; data6_j(i+4,1)];
    roboty = [roboty; data6_j(i+5,1)];
    % portmotor = [portmotor; data6_j(i+6,1)];
    % starboardmotor = [starboardmotor; data6_j(i+7,1)];
    i = i+6;
end

figure
hold on
scatter(robotx,roboty)
scatter(goalx, goaly)
scatter([259, 360], [183, 311])
for i = 1:length(azimuth)
    plot([robotx(i), robotx(i)+15*cos(azimuth(i)*(pi/180))],[roboty(i), roboty(i)+15*sin(azimuth(i)*(pi/180))])
    plot([robotx(i), robotx(i)+30*cos(heading(i)*(pi/180))],[roboty(i), roboty(i)+30*sin(heading(i)*(pi/180))])

end
% xlim([0, 396])
% ylim([0, 244])
title("Reference Positioning Test Canada")
xlabel("x position (cm)")
ylabel("y position (cm)")
legend("Robot Position")
axis equal
hold off

