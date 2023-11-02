%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: C:\Users\xsedla1n\Desktop\ntc.csv
%
% Auto-generated by MATLAB on 02-Nov-2023 15:10:05

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["t", "r"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable("C:\Temp\Macejko\embeded\CV_06\ntc.csv", opts);

%% Convert to output type
t = tbl.t;
r = tbl.r;

%% Clear temporary variables
clear opts tbl

%% Processing
ad = (r ./ (r + 10)) * 2^10;
p = polyfit(ad, t, 10);

ad2 = 0:1023;
t2 = round(polyval(p, ad2), 1);

figure;
plot(ad, t, 'bo');
hold on;
plot(ad2, t2, 'r');

%% Generating data
dlmwrite('data.dlm', t2*10, ',');


