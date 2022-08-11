% This script collects the data used to build the tables 
clear all; close all; clc;
folderPath = 'C:\Users\ataye\OneDrive - MathWorks\Desktop\Softwares\Safe and Scalable UAM Trajectory Planner\data\Experiements\2 agents\';
%initialize arrays for data storage

experimentData = [];

for experiment = 1:25
    % run the experiment
    [totalNMACs,stepTimer] = main;

    % collect data for building the table

    allNMAC = totalNMACs;
    Mean = mean(stepTimer);
    Median = median(stepTimer);
    Std = std(stepTimer);
    Throughput = sum(stepTimer);

    experimentData = [experimentData;[allNMAC,Mean,Median,Std,Throughput] ];
    % save the workspace for future reference 
    filePath = strcat(folderPath,'Experiment ',num2str(experiment));
    save(filePath);
end
