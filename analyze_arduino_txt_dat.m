clear all; close all; clc;
% Specify the directory and file pattern for the log files
logFiles = dir('*.txt');  % This retrieves all .log files in the current directory
figure;
% Loop through each log file
for k = 1:length(logFiles)
    % Load and parse the reflow log file
    filename = logFiles(k).name;

    fileID = fopen(filename, 'r');
    rawText = textscan(fileID, '%s', 'Delimiter', '\n');
    fclose(fileID);
    lines = rawText{1};
    
    % Initialize arrays
    time = [];
    dT = [];
    temp = [];
    target = [];
    output = [];
    
    % Pattern to match lines like: Time: 0.16 s, Temp: 29.41 C, Target: 50.15 C, Output: 120.51
    expr = 'Time:\s*([\d.]+)\s*s,\s*dT:\s*([\d.]+)\s*s,\s*Temp:\s*([\d.]+)\s*C,\s*Target:\s*([\d.]+)\s*C,\s*Output:\s*([\d.]+)\s*';
    for i = 1:length(lines)
        line = lines{i};
        tokens = regexp(line, expr, 'tokens');
        if isempty(tokens) || numel(tokens{1}) < 5
            continue
        end
        nums = str2double(tokens{1});
        time(end+1) = nums(1);
        dT(end+1) = nums(2);
        temp(end+1) = nums(3);
        target(end+1) = nums(4);
        output(end+1) = nums(5);
    end
    
    % Find the transition point between heating and cooling
    [~, maxIdx] = max(target);
    
    % Plot Heating Phase
    figure(k)
    plot(time, temp, 'r', 'LineWidth', 1.5); hold on;
    plot(time, target, 'k--');
    plot(time, output, 'b:');
    xlabel('Time (s)');
    ylabel('Value');
    % Replace underscores and other special characters
    filename = strrep(filename, '_', '\_');  % Escape underscores
    filename = strrep(filename, '#', '\#');  % Escape hash
    title(filename);
    legend('Temperature (C)', 'Target (C)', 'Output');
    grid on;
end