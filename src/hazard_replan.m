%% hazard_replan.m
% SlopeGuard CW2 - hazard injection and replanning
% Author: Tomiwa Adeyemi
%
% Simulates a mid-mission hazard event: the drone detects a secondary
% slip and updates the shared risk map. The UGV must replan to avoid
% the new hazard. We compare the original A* path vs the replanned path
% on metrics: path length, mean risk, nodes expanded, replan latency.

clear; clc; close all;

%% Load environment and original path
projectRoot = 'C:\Users\tomiw\OneDrive - University of Hertfordshire\MATLAB\CW2 Slopeguard';
load(fullfile(projectRoot,'results','environment.mat'));
load(fullfile(projectRoot,'results','astar_path.mat'));

[nRows, nCols] = size(costMap);
originalPath   = path;
originalLength = pathLength;
originalRisk   = pathRiskMean;
originalNodes  = nodesExpanded;
originalTime   = planTime;

%% Inject hazard - secondary slip detected by drone
% Region chosen to intersect the original path's exit from the safe
% corridor, forcing a genuine detour.
hazardCostMap = costMap;
hazardRows = 25:40;
hazardCols = 15:30;
hazardCostMap(hazardRows, hazardCols) = 0.92;

fprintf('Hazard injected: rows %d-%d, cols %d-%d\n', ...
    min(hazardRows), max(hazardRows), min(hazardCols), max(hazardCols));
fprintf('Original path metrics:\n');
fprintf('  length=%.2f m, mean risk=%.3f, nodes=%d, time=%.4f s\n\n', ...
    originalLength/resolution, originalRisk, originalNodes, originalTime);

%% Replan from start with updated map (full A* re-search)
w_d = 1.0; w_r = 5.0; maxCost = 0.95;

tic;
[newPath, newNodes, newFound] = riskAwareAStar( ...
    hazardCostMap, startPos, goalPos, w_d, w_r, maxCost);
newTime = toc;

if ~newFound
    error('Replan failed - no path around hazard.');
end

%% Compute new metrics
newLength = sum(sqrt(sum(diff(newPath).^2, 2)));
newRiskSum = 0;
for i = 1:size(newPath,1)
    newRiskSum = newRiskSum + hazardCostMap(newPath(i,1), newPath(i,2));
end
newRisk = newRiskSum / size(newPath,1);

fprintf('Replanned path metrics:\n');
fprintf('  length=%.2f m, mean risk=%.3f, nodes=%d, time=%.4f s\n\n', ...
    newLength/resolution, newRisk, newNodes, newTime);

%% Comparison summary
fprintf('--- Comparison ---\n');
fprintf('  Path length     : %.2f m -> %.2f m  (%+.1f%%)\n', ...
    originalLength/resolution, newLength/resolution, ...
    100*(newLength-originalLength)/originalLength);
fprintf('  Mean cell risk  : %.3f  -> %.3f\n', originalRisk, newRisk);
fprintf('  Nodes expanded  : %d   -> %d\n', originalNodes, newNodes);
fprintf('  Planning time   : %.4f s -> %.4f s\n', originalTime, newTime);

%% Visualise both paths on the updated map
figure('Name','Hazard injection + replan','Color','w','Position',[100 100 900 700]);
imagesc(hazardCostMap);
colormap(flipud(hot));
colorbar;
axis equal tight;
set(gca,'YDir','normal');
hold on;
% Highlight hazard region with dashed magenta outline
rectangle('Position',[min(hazardCols)-0.5, min(hazardRows)-0.5, ...
    max(hazardCols)-min(hazardCols)+1, max(hazardRows)-min(hazardRows)+1], ...
    'EdgeColor','m','LineStyle','--','LineWidth',2);
plot(originalPath(:,2), originalPath(:,1), 'b--', 'LineWidth', 2);
plot(newPath(:,2),      newPath(:,1),      'g-',  'LineWidth', 2.5);
plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goalPos(2),  goalPos(1),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
hHaz = rectangle('Position',[min(hazardCols)-0.5, min(hazardRows)-0.5, ...
    max(hazardCols)-min(hazardCols)+1, max(hazardRows)-min(hazardRows)+1], ...
    'EdgeColor','m','LineStyle','--','LineWidth',2);
hOrig = plot(originalPath(:,2), originalPath(:,1), 'b--', 'LineWidth', 2);
hNew  = plot(newPath(:,2),      newPath(:,1),      'g-',  'LineWidth', 2.5);
hSt   = plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
hGo   = plot(goalPos(2),  goalPos(1),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
legend([hOrig hNew hSt hGo],{'Original path','Replanned path','Start','Goal'}, ...
    'Location','northwest');
title('Hazard injection: original (blue dashed) vs replanned (green)');
xlabel('x (cells, 0.25 m each)');
ylabel('y (cells, 0.25 m each)');

%% Save replan results
save(fullfile(projectRoot,'results','hazard_replan.mat'), ...
    'originalPath','newPath','hazardCostMap','hazardRows','hazardCols', ...
    'originalLength','newLength','originalRisk','newRisk', ...
    'originalNodes','newNodes','originalTime','newTime');
fprintf('\nSaved comparison data.\n');

%% --- Helper: A* as a function (same algorithm as Step 2) ---
function [path, nodesExpanded, found] = riskAwareAStar(costMap, startPos, goalPos, w_d, w_r, maxCost)
    [nRows, nCols] = size(costMap);
    closedSet = false(nRows, nCols);
    gScore = inf(nRows, nCols);
    parentRow = zeros(nRows, nCols);
    parentCol = zeros(nRows, nCols);
    gScore(startPos(1), startPos(2)) = 0;
    hStart = sqrt((startPos(1)-goalPos(1))^2 + (startPos(2)-goalPos(2))^2);
    openList = [startPos(1), startPos(2), 0, hStart];
    neighbourOffsets = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    nodesExpanded = 0;
    found = false;

    while ~isempty(openList)
        [~, idx] = min(openList(:,4));
        current = openList(idx,:);
        openList(idx,:) = [];
        cr = current(1); cc = current(2);
        if cr == goalPos(1) && cc == goalPos(2)
            found = true; break;
        end
        if closedSet(cr,cc), continue; end
        closedSet(cr,cc) = true;
        nodesExpanded = nodesExpanded + 1;

        for k = 1:size(neighbourOffsets,1)
            nr = cr + neighbourOffsets(k,1);
            nc = cc + neighbourOffsets(k,2);
            if nr<1||nr>nRows||nc<1||nc>nCols, continue; end
            if closedSet(nr,nc), continue; end
            if costMap(nr,nc) >= maxCost, continue; end
            stepDist = sqrt((cr-nr)^2 + (cc-nc)^2);
            edgeCost = w_d*stepDist + w_r*costMap(nr,nc);
            tentativeG = gScore(cr,cc) + edgeCost;
            if tentativeG < gScore(nr,nc)
                gScore(nr,nc) = tentativeG;
                parentRow(nr,nc) = cr;
                parentCol(nr,nc) = cc;
                h = sqrt((nr-goalPos(1))^2 + (nc-goalPos(2))^2);
                openList = [openList; nr, nc, tentativeG, tentativeG+h];
            end
        end
    end

    path = [];
    if found
        path = goalPos;
        cur = goalPos;
        while ~isequal(cur, startPos)
            pr = parentRow(cur(1), cur(2));
            pc = parentCol(cur(1), cur(2));
            cur = [pr, pc];
            path = [cur; path];
        end
    end
end