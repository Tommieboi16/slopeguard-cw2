%% build_environment.m
% SlopeGuard CW2 - debris field cost map
% Author: Tomiwa Adeyemi

clear; clc; close all;

%% Project root (absolute path - bulletproof against folder confusion)
projectRoot = 'C:\Users\tomiw\OneDrive - University of Hertfordshire\MATLAB\CW2 Slopeguard';

%% Map parameters
mapWidth   = 25;
mapHeight  = 25;
resolution = 4;
nRows = mapHeight * resolution;
nCols = mapWidth  * resolution;

%% Initialise base traversability cost
costMap = 0.1 * ones(nRows, nCols);

%% Terrain features
for i = 1:nRows
    for j = 1:nCols
        if abs(i - j) < 6 && i > 20 && i < 80
            costMap(i, j) = 0.85;
        end
    end
end
costMap(40:65, 55:85) = 0.65;
costMap(20:28, 70:78) = 1.0;
costMap(75:85, 30:40) = 1.0;
costMap(:, 5:15) = 0.05;

victimMap = zeros(nRows, nCols);
victimMap(80:90, 80:90) = 1.0;

startPos = [10, 10];
goalPos  = [85, 85];

%% Visualise
figure('Name','SlopeGuard cost map','Color','w');
imagesc(costMap); 
colormap(flipud(hot)); 
colorbar;
axis equal tight; 
set(gca,'YDir','normal');
hold on;
plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goalPos(2),  goalPos(1),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
[vr, vc] = find(victimMap > 0);
plot(vc, vr, 'c.', 'MarkerSize', 4);
legend('Start','Goal','Victim cue','Location','northwest');
title('SlopeGuard debris field - risk-weighted cost map');
xlabel('x (cells, 0.25 m each)'); 
ylabel('y (cells, 0.25 m each)');

%% Save environment - with error checking
resultsDir = fullfile(projectRoot,'results');
if ~exist(resultsDir,'dir')
    mkdir(resultsDir);
end
savePath = fullfile(resultsDir,'environment.mat');

try
    save(savePath, 'costMap','victimMap','startPos','goalPos','resolution');
    fprintf('Environment built: %d x %d cells (%.1f x %.1f m)\n', nRows, nCols, mapWidth, mapHeight);
    fprintf('Saved to: %s\n', savePath);
catch ME
    fprintf(2,'SAVE FAILED: %s\n', ME.message);
end