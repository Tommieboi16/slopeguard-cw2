%% experiments.m
% SlopeGuard CW2 - multi-trial experiments
% Author: Tomiwa Adeyemi
%
% Runs A* on 5 scenarios:
%  1. Baseline (no hazard)
%  2-4. Three different hazard locations
%  5. Ablation: risk weighting disabled
% Records metrics for each and produces a comparison figure.

clear; clc; close all;

%% Load environment
projectRoot = 'C:\Users\tomiw\OneDrive - University of Hertfordshire\MATLAB\CW2 Slopeguard';
load(fullfile(projectRoot,'results','environment.mat'));
[nRows, nCols] = size(costMap);

%% Define experiments
trials(1).name = 'Baseline (no hazard)';
trials(1).map  = costMap;
trials(1).w_d  = 1.0; trials(1).w_r = 5.0; trials(1).maxCost = 0.95;

trials(2).name = 'Hazard A: corridor exit';
m2 = costMap; m2(25:40, 15:30) = 0.92;
trials(2).map  = m2;
trials(2).w_d  = 1.0; trials(2).w_r = 5.0; trials(2).maxCost = 0.95;

trials(3).name = 'Hazard B: mid-map';
m3 = costMap; m3(45:60, 30:45) = 0.92;
trials(3).map  = m3;
trials(3).w_d  = 1.0; trials(3).w_r = 5.0; trials(3).maxCost = 0.95;

trials(4).name = 'Hazard C: near goal';
m4 = costMap; m4(70:82, 60:75) = 0.92;
trials(4).map  = m4;
trials(4).w_d  = 1.0; trials(4).w_r = 5.0; trials(4).maxCost = 0.95;

trials(5).name = 'Ablation: w_r = 0 (no risk weighting)';
trials(5).map  = costMap;
trials(5).w_d  = 1.0; trials(5).w_r = 0.0; trials(5).maxCost = 0.95;

%% Run all trials
fprintf('--- Running %d trials ---\n\n', numel(trials));
results = struct();

for i = 1:numel(trials)
    tr = trials(i);
    fprintf('Trial %d: %s\n', i, tr.name);

    tic;
    [pth, nodes, found] = riskAwareAStar( ...
        tr.map, startPos, goalPos, tr.w_d, tr.w_r, tr.maxCost);
    elapsed = toc;

    if ~found
        warning('Trial %d failed - no path.', i);
        results(i).name = tr.name;
        results(i).found = false;
        continue;
    end

    % Path metrics on the same map the planner used
    L = sum(sqrt(sum(diff(pth).^2, 2)));
    riskSum = 0;
    for k = 1:size(pth,1)
        riskSum = riskSum + tr.map(pth(k,1), pth(k,2));
    end
    meanRisk = riskSum / size(pth,1);

    % Also evaluate path safety on the BASE costMap (what was "really" there)
    % For the ablation trial this exposes whether ignoring risk produces
    % a path that crosses dangerous terrain.
    realRiskSum = 0;
    for k = 1:size(pth,1)
        realRiskSum = realRiskSum + costMap(pth(k,1), pth(k,2));
    end
    realMeanRisk = realRiskSum / size(pth,1);

    results(i).name        = tr.name;
    results(i).found       = true;
    results(i).path        = pth;
    results(i).map         = tr.map;
    results(i).length_m    = L / resolution;
    results(i).meanRisk    = meanRisk;
    results(i).realMeanRisk = realMeanRisk;
    results(i).nodes       = nodes;
    results(i).time_s      = elapsed;

    fprintf('  length=%.2f m  meanRisk=%.3f  realRisk=%.3f  nodes=%d  time=%.4f s\n\n', ...
        L/resolution, meanRisk, realMeanRisk, nodes, elapsed);
end

%% Print results table
fprintf('\n=========================================================================\n');
fprintf('%-38s %8s %10s %10s %8s %10s\n', ...
    'Trial','Length(m)','MeanRisk','RealRisk','Nodes','Time(s)');
fprintf('=========================================================================\n');
for i = 1:numel(results)
    if results(i).found
        fprintf('%-38s %8.2f %10.3f %10.3f %8d %10.4f\n', ...
            results(i).name, results(i).length_m, results(i).meanRisk, ...
            results(i).realMeanRisk, results(i).nodes, results(i).time_s);
    else
        fprintf('%-38s %s\n', results(i).name, '<<no path>>');
    end
end
fprintf('=========================================================================\n');

%% Composite figure: all paths overlaid
figure('Name','Trial paths','Color','w','Position',[100 100 1100 700]);
trialColours = {'g','b','m','c','r'};
trialStyles  = {'-','-','-','-','--'};

% Use the BASE costMap as background so terrain is consistent
imagesc(costMap);
colormap(flipud(hot)); colorbar;
axis equal tight; set(gca,'YDir','normal');
hold on;

handles = gobjects(numel(results),1);
labels  = cell(numel(results),1);
for i = 1:numel(results)
    if results(i).found
        handles(i) = plot(results(i).path(:,2), results(i).path(:,1), ...
            'Color', trialColours{i}, 'LineStyle', trialStyles{i}, 'LineWidth', 2);
        labels{i} = results(i).name;
    end
end
plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goalPos(2),  goalPos(1),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
valid = arrayfun(@(h) isgraphics(h), handles);
legend(handles(valid), labels(valid), 'Location','southoutside', 'NumColumns', 2);
title('A* paths across trials (overlaid on base cost map)');
xlabel('x (cells, 0.25 m each)'); ylabel('y (cells, 0.25 m each)');

%% Bar chart: latency and nodes
foundIdx = find(arrayfun(@(r) r.found, results));
figure('Name','Trial metrics','Color','w','Position',[100 100 1000 500]);

subplot(1,2,1);
bar(arrayfun(@(i) results(i).time_s*1000, foundIdx));
set(gca,'XTickLabel',{results(foundIdx).name},'XTickLabelRotation',30);
ylabel('Planning time (ms)');
title('Replanning latency per trial');
grid on;

subplot(1,2,2);
bar([arrayfun(@(i) results(i).meanRisk, foundIdx)' ...
     arrayfun(@(i) results(i).realMeanRisk, foundIdx)']);
set(gca,'XTickLabel',{results(foundIdx).name},'XTickLabelRotation',30);
ylabel('Mean cell risk');
legend('Planner-evaluated risk','Real (base map) risk','Location','northwest');
title('Path risk per trial');
grid on;

%% Save results
save(fullfile(projectRoot,'results','experiments.mat'),'results','trials');
fprintf('\nSaved experiment results.\n');

%% --- Helper: A* (same as Step 3) ---
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
    nodesExpanded = 0; found = false;

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