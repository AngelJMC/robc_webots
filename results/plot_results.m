%% Import data from text file.
% Script for importing data from the following text file:
%
%    /home/angel/Proyectos/robc_webots/simulation_results-1561047479.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2019/06/20 19:31:21

%% Initialize variables.
filename = '/home/angel/Proyectos/robc_webots/results/simulation_results-1561047479.scv';
delimiter = ',';
startRow = 2;

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
simulationresults1561047479 = table(dataArray{1:end-1}, 'VariableNames', {'Kp','Ki','Sa','Sb','Bp','distance','bestdist'});

%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%% PLot 

figure(1)
plot(simulationresults1561047479.Kp )
hold on
plot(simulationresults1561047479.Ki )
plot(simulationresults1561047479.Sa )
plot(simulationresults1561047479.Sb )
plot(simulationresults1561047479.Bp )

figure(2)
plot(simulationresults1561047479.distance, '.' )
hold on
plot(simulationresults1561047479.bestdist )