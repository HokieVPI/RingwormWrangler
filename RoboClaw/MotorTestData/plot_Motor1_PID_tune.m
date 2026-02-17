%% Plot Motor1_PID_tune.csv - Each column vs timestamp
% Reads CSV, separates columns into named arrays, plots each vs Time

clear; clc; close all;

%% Load CSV
csvFile = fullfile(fileparts(mfilename('fullpath')), 'Motor1_PID_tune.csv');
T = readtable(csvFile);

%% Get column names and extract Time
colNames = T.Properties.VariableNames;
Time = T.Time;

%% Create separate arrays named from column headers (skip Time)
% Sanitize names for valid MATLAB identifiers: replace invalid chars with underscore
nCols = width(T);
for k = 1:nCols
    rawName = colNames{k};
    % Make valid variable name (letters, digits, underscore only)
    safeName = regexprep(rawName, '\W', '_');
    if strcmp(rawName, 'Time')
        continue  % Time already assigned
    end
    eval([safeName ' = T.' rawName ';']);
end

%% Optional: also store in a struct for easy access (col name -> array)
Data = struct();
Data.Time = Time;
for k = 2:nCols
    rawName = colNames{k};
    safeName = regexprep(rawName, '\W', '_');
    Data.(safeName) = T.(rawName);
end

%% Graph each column vs timestamp
% One figure with subplots (one subplot per data column)
nDataCols = nCols - 1;  % exclude Time
nRows = ceil(sqrt(nDataCols));
nColsPlot = ceil(nDataCols / nRows);

figure('Name', 'Motor1 PID Tune - All channels vs Time', 'NumberTitle', 'off');
for k = 2:nCols
    idx = k - 1;
    subplot(nRows, nColsPlot, idx);
    plot(Time, T.(colNames{k}), 'LineWidth', 0.5);
    title(colNames{k}, 'Interpreter', 'none');
    xlabel('Time (s)');
    ylabel(colNames{k}, 'Interpreter', 'none');
    grid on;
end
sgtitle('Motor1\_PID\_tune: each column vs Time');

%% Optional: single full-size plot of selected channels
% % Time_tune=Time(2083:3169); 
% figure;
% plot(Time, M1Speed,'b');
% title('Speed channels vs Time');
% xlabel('Time (s)');
% ylabel('QPPS')
% xlim([94 100])
% grid on;
% hold on
% % plot(Time,M1ISpeed,'c')
% plot(Time,M1Pos)
% yyaxis("right")
% plot(Time, M1Current, 'r')
% ylabel('Current Amps')
% legend('Speed','Angular Position','Current (Amps)')
% hold off 
%% 
figure;
plot(Time, M1Speed,'b');
title('Speed channels vs Time');
xlabel('Time (s)');
ylabel('QPPS (count/s')
xlim([96.8 97.8])
grid on;
hold on
% plot(Time,M1ISpeed,'c')
plot(Time,M1PWM,'m --')
yyaxis("right")
plot(Time, M1Pos, 'r')
ylabel('Position (count) ')
legend('Vel','PWM','Pos')
hold off 
%%
% figure;
% plot(Time, M1Speed,'b');
% title('Speed channels vs Time');
% xlabel('Time (s)');
% ylabel('QPPS')
% xlim([94 100])
% grid on;
% hold on
% % plot(Time,M1ISpeed,'c')
% plot(Time,M1Pos)
% yyaxis("right")
% plot(Time, M1Current, 'r')
% ylabel('Current Amps')
% legend('Speed','Angular Position','Current (Amps)')
% hold off 