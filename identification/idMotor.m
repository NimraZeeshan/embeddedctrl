% Identyfing System using Least Square Methods

%% Parameter Initialization

samplingTime = 0.1;

%% Dataset Reading

identification = csvread('datasetidentifiation.csv', 2:end, :);

in = identification(:, 1); % Input
out = identification(:, 2); % Output

%% Split the Dataset into Identification set and Validation set

% Get the data id
dataId = iddata(out, in, samplingTime);

ident = dtrend(dataId(1:400));
val = dtrend(dataId(401:end)); 

%% Identification Process

% Correlation analysis
ca = impulseest(ident, 197);
% ARX Model
arxx = arx(ident, [2 3 0]);

%% Compare the results

compare(val, ca, arxx);

% In this case the correlation analysis fit is better than ARX method
% To get the better results attempt to change the parameter such as the
% order of the ARX and CA or the dataset split proportion