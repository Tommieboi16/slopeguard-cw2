%% run_simulation.m
% SlopeGuard CW2 - drive a differential-drive UGV along the replanned path
% Author: Tomiwa Adeyemi
%
% Loads the path from hazard_replan.m and uses Pure Pursuit

clear; clc; close all;

%% Load environment and replanned path
projectRoot = 'C:\Users\tomiw\OneDrive - University of Hertfordshire\MATLAB\CW2 Slopeguard';
load(fullfile(projectRoot,'results','environment.mat'));
load(fullfile(projectRoot,'results','hazard_replan.mat'));

% Convert path from grid (row,col) to world (x,y) in metres
% In the figure: x = col / resolution, y = row / resolution
pathXY  = [newPath(:,2), newPath(:,1)] / resolution;
startXY = [startPos(2), startPos(1)] / resolution;
goalXY  = [goalPos(2),  goalPos(1)]  / resolution;

%% Robot model - differential drive (Spot-like footprint)
robot = differentialDriveKinematics( ...
    "TrackWidth", 0.5, ...
    "VehicleInputs", "VehicleSpeedHeadingRate");

%% Pure Pursuit controller
controller = controllerPurePursuit;
controller.Waypoints             = pathXY;
controller.DesiredLinearVelocity = 0.5;   % m/s
controller.MaxAngularVelocity    = 1.5;   % rad/s
controller.LookaheadDistance     = 0.6;   % m

%% Simulation settings
sampleTime = 0.1;                    % 10 Hz controller update
simTime    = 0:sampleTime:120;       % up to 120 s
goalRadius = 0.3;                    % stop when within this radius

% Initial state: pose at start, heading toward first waypoint
initHeading = atan2(pathXY(2,2)-startXY(2), pathXY(2,1)-startXY(1));
pose = [startXY(1); startXY(2); initHeading];

trajectory = pose';                  % log of robot positions

%% Visualisation setup
figure('Name','UGV simulation','Color','w','Position',[100 100 900 700]);
imagesc([0 size(hazardCostMap,2)/resolution], ...
        [0 size(hazardCostMap,1)/resolution], hazardCostMap);
colormap(flipud(hot)); colorbar;
axis equal tight; set(gca,'YDir','normal');
hold on;

% Hazard outline
rectangle('Position',[(min(hazardCols)-0.5)/resolution, ...
    (min(hazardRows)-0.5)/resolution, ...
    (max(hazardCols)-min(hazardCols)+1)/resolution, ...
    (max(hazardRows)-min(hazardRows)+1)/resolution], ...
    'EdgeColor','m','LineStyle','--','LineWidth',2);

hPath  = plot(pathXY(:,1), pathXY(:,2), 'g-', 'LineWidth', 2);
hSt    = plot(startXY(1), startXY(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
hGo    = plot(goalXY(1),  goalXY(2),  'b*', 'MarkerSize', 14, 'LineWidth', 2);
hRobot = plot(pose(1), pose(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor','b');
hTrail = plot(pose(1), pose(2), 'b-', 'LineWidth', 1.5);

title('SlopeGuard UGV following replanned path');
xlabel('x (m)'); ylabel('y (m)');
legend([hPath hSt hGo hRobot hTrail], ...
    {'Planned path','Start','Goal','Robot','Trail'}, ...
    'Location','northwest');
drawnow;

%% Simulation loop
fprintf('Starting UGV simulation...\n');
goalReached = false;
for t = simTime
    % Distance to goal
    distToGoal = norm(pose(1:2)' - goalXY);
    if distToGoal < goalRadius
        goalReached = true;
        fprintf('Goal reached at t = %.2f s (%.2f m from goal)\n', t, distToGoal);
        break;
    end

    % Get velocity command from Pure Pursuit
    [v, omega] = controller(pose);

    % Step robot kinematics (forward Euler)
    velCmd = [v; omega];
    dPose  = derivative(robot, pose, velCmd);
    pose   = pose + dPose * sampleTime;

    % Log trajectory
    trajectory = [trajectory; pose'];

    % Update animation every 5 ticks (~0.5s)
    if mod(round(t/sampleTime),5) == 0
        set(hRobot,'XData',pose(1),'YData',pose(2));
        set(hTrail,'XData',trajectory(:,1),'YData',trajectory(:,2));
        drawnow limitrate;
    end
end

if ~goalReached
    fprintf('Simulation timed out before reaching goal.\n');
end

%% Final update + metrics
set(hRobot,'XData',pose(1),'YData',pose(2));
set(hTrail,'XData',trajectory(:,1),'YData',trajectory(:,2));

travelled = sum(sqrt(sum(diff(trajectory(:,1:2)).^2,2)));
fprintf('\nSimulation summary:\n');
fprintf('  Travelled distance : %.2f m\n', travelled);
fprintf('  Planned path length: %.2f m\n', newLength/resolution);
fprintf('  Tracking overshoot : %.1f %%\n', ...
    100*(travelled - newLength/resolution)/(newLength/resolution));
fprintf('  Final distance to goal: %.2f m\n', norm(pose(1:2)' - goalXY));

%% Save trajectory
save(fullfile(projectRoot,'results','trajectory.mat'), ...
    'trajectory','pathXY','startXY','goalXY','sampleTime');
fprintf('Trajectory saved.\n');