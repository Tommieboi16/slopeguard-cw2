%% risk_aware_astar.m
% SlopeGuard CW2 - Risk-weighted A* global planner
% Author: Tomiwa Adeyemi
%
% Implements A* search (Hart, Nilsson & Raphael, 1968) on the
% risk-weighted cost map produced by build_environment.m.
%
% The cost function J(c) is a simplified form of the full SlopeGuard
% expression from CW1: J(c) = w_d*d + w_s*s + w_r*r
%   d = step distance (Euclidean between adjacent cells)
%   s = slope severity (high cost map values represent steep terrain)
%   r = terrain roughness (also encoded in cost map values)
% In this implementation, slope and roughness are fused into a single
% per-cell traversability cost (the loaded costMap), and weights are
% applied per move. See report Section 3 for the full derivation.

clear; clc; close all;

%% Load the environment
projectRoot = 'C:\Users\tomiw\OneDrive - University of Hertfordshire\MATLAB\CW2 Slopeguard';
load(fullfile(projectRoot,'results','environment.mat'));

[nRows, nCols] = size(costMap);

%% Planner parameters (cost-function weights)
w_d = 1.0;     % weight on step distance
w_r = 5.0;     % weight on terrain risk (slope + roughness combined)
maxCost = 0.95; % cells with cost above this are treated as impassable

fprintf('Risk-weighted A* planner\n');
fprintf('Start: [%d, %d]   Goal: [%d, %d]\n', startPos, goalPos);
fprintf('Weights: w_d=%.1f  w_r=%.1f  maxCost=%.2f\n\n', w_d, w_r, maxCost);

%% A* search
tic;  % start timer for replanning latency analysis later

% Each node stored as: [row, col, gScore, fScore, parentIdx]
% Use linear indexing for fast lookup in closed set
openList   = [];                    % candidate nodes to expand
closedSet  = false(nRows, nCols);   % expanded nodes
gScore     = inf(nRows, nCols);     % cost from start to each cell
parentRow  = zeros(nRows, nCols);   % for path reconstruction
parentCol  = zeros(nRows, nCols);

% Initialise with start node
gScore(startPos(1), startPos(2)) = 0;
hStart = euclideanDist(startPos, goalPos);
openList = [startPos(1), startPos(2), 0, hStart];

% 8-connected neighbours: up, down, left, right, and 4 diagonals
neighbourOffsets = [ -1  0;  1  0;  0 -1;  0  1; ...
                     -1 -1; -1  1;  1 -1;  1  1 ];

nodesExpanded = 0;
pathFound = false;

while ~isempty(openList)
    % Pick node with lowest fScore from open list
    [~, idx] = min(openList(:,4));
    current = openList(idx,:);
    openList(idx,:) = [];           % remove from open list
    
    cr = current(1); cc = current(2);
    
    % Goal check
    if cr == goalPos(1) && cc == goalPos(2)
        pathFound = true;
        break;
    end
    
    % Skip if already expanded
    if closedSet(cr, cc)
        continue;
    end
    closedSet(cr, cc) = true;
    nodesExpanded = nodesExpanded + 1;
    
    % Expand neighbours
    for k = 1:size(neighbourOffsets,1)
        nr = cr + neighbourOffsets(k,1);
        nc = cc + neighbourOffsets(k,2);
        
        % Bounds check
        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue;
        end
        
        % Skip if already expanded
        if closedSet(nr, nc)
            continue;
        end
        
        % Skip impassable cells
        if costMap(nr, nc) >= maxCost
            continue;
        end
        
        % Risk-weighted edge cost: J(c) = w_d*d + w_r*r
        stepDist = euclideanDist([cr,cc], [nr,nc]);
        riskCost = costMap(nr, nc);
        edgeCost = w_d*stepDist + w_r*riskCost;
        
        tentativeG = gScore(cr,cc) + edgeCost;
        
        if tentativeG < gScore(nr, nc)
            gScore(nr, nc) = tentativeG;
            parentRow(nr, nc) = cr;
            parentCol(nr, nc) = cc;
            
            h = euclideanDist([nr,nc], goalPos);
            f = tentativeG + h;
            openList = [openList; nr, nc, tentativeG, f];
        end
    end
end

planTime = toc;

%% Reconstruct path
if pathFound
    path = goalPos;
    cur = goalPos;
    while ~isequal(cur, startPos)
        pr = parentRow(cur(1), cur(2));
        pc = parentCol(cur(1), cur(2));
        if pr == 0 && pc == 0
            error('Path reconstruction failed - parent missing');
        end
        cur = [pr, pc];
        path = [cur; path];
    end
    
    pathLength = sum(sqrt(sum(diff(path).^2, 2)));
    pathRiskSum = 0;
    for i = 1:size(path,1)
        pathRiskSum = pathRiskSum + costMap(path(i,1), path(i,2));
    end
    pathRiskMean = pathRiskSum / size(path,1);
    
    fprintf('PATH FOUND\n');
    fprintf('  Nodes expanded   : %d\n', nodesExpanded);
    fprintf('  Path length      : %.2f cells (%.2f m)\n', pathLength, pathLength/resolution);
    fprintf('  Mean cell risk   : %.3f\n', pathRiskMean);
    fprintf('  Planning time    : %.4f s\n', planTime);
else
    error('No path found - check map or weights.');
end

%% Visualise path on cost map
figure('Name','A* path on cost map','Color','w');
imagesc(costMap); 
colormap(flipud(hot)); 
colorbar;
axis equal tight; 
set(gca,'YDir','normal');
hold on;
plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2.5);
plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goalPos(2),  goalPos(1),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
[vr, vc] = find(victimMap > 0);
plot(vc, vr, 'c.', 'MarkerSize', 4);
legend('Planned path','Start','Goal','Victim cue','Location','northwest');
title(sprintf('Risk-weighted A* path  (length %.1fm, mean risk %.2f)', ...
    pathLength/resolution, pathRiskMean));
xlabel('x (cells, 0.25 m each)'); 
ylabel('y (cells, 0.25 m each)');

%% Save the planned path and metrics
savePath = fullfile(projectRoot,'results','astar_path.mat');
save(savePath, 'path','pathLength','pathRiskMean','planTime','nodesExpanded', ...
    'w_d','w_r','maxCost');
fprintf('\nSaved path to: %s\n', savePath);

%% Helper function (must be at end of script in MATLAB)
function d = euclideanDist(a, b)
    d = sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);
end