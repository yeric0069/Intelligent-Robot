clear;
close all;
clc;

map = binaryOccupancyMap(25, 25, 10);

% lines
for index=6:0.01:22
    setOccupancy(map, [1, index], 1)
end

for index=1:0.01:6
    setOccupancy(map, [index, 6], 1)
end

for index=1:0.01:6
    setOccupancy(map, [index, 10], 1)
end

for index=1:0.01:6
    setOccupancy(map, [index, 14], 1)
end

for index=1:0.01:6
    setOccupancy(map, [index, 18], 1)
end

for index=1:0.01:6
    setOccupancy(map, [index, 22], 1)
end

for index=12:0.01:17
    setOccupancy(map, [index, 6], 1)
end

for index=12:0.01:17
    setOccupancy(map, [index, 10], 1)
end

for index=12:0.01:17
    setOccupancy(map, [index, 14], 1)
end

for index=12:0.01:17
    setOccupancy(map, [index, 18], 1)
end

for index=12:0.01:17
    setOccupancy(map, [index, 22], 1)
end

for index=6:0.01:22
    setOccupancy(map, [17, index], 1)
end

% cars
[x,y] = meshgrid(161:179,20:44); % [2 7.1 2.4 1.8]
setOccupancy(map, [x(:) y(:)], 1, "grid")

[x,y] = meshgrid(121:139,20:44); % [2 11.1 2.4 1.8]
setOccupancy(map, [x(:) y(:)], 1, "grid")

[x,y] = meshgrid(41:59,20:44); % [2 19.1 2.4 1.8]
setOccupancy(map, [x(:) y(:)], 1, "grid")

[x,y] = meshgrid(161:179,130:154); % [13 7.1 2.4 1.8]
setOccupancy(map, [x(:) y(:)], 1, "grid")

[x,y] = meshgrid(41:59,130:154); % [13 19.1 2.4 1.8]
setOccupancy(map, [x(:) y(:)], 1, "grid")

[x,y] = meshgrid(148:172,81:99); % [8.1 7.8 1.8 2.4]
setOccupancy(map, [x(:) y(:)], 1, "grid")

inflate(map, 1.1)

figure
show(map)

hold on;

line([1; 1], [6; 22])
line([1; 6], [6; 6])
line([1; 6], [10; 10])
line([1; 6], [14; 14])
line([1; 6], [18; 18])

line([1; 6], [22; 22])
line([12; 17], [6; 6])
line([12; 17], [10; 10])
line([12; 17], [14; 14])
line([12; 17], [18; 18])
line([17; 17], [6; 22])
line([17; 12], [22; 22])

rectangle('Position', [2 7.1 2.4 1.8])
rectangle('Position', [2 11.1 2.4 1.8])
rectangle('Position', [2 19.1 2.4 1.8])

rectangle('Position', [13 7.1 2.4 1.8])
rectangle('Position', [13 19.1 2.4 1.8])

rectangle('Position', [8.1 7.8 1.8 2.4])

% initial position
rectangle('Position', [0 2.1 2.4 1.8])

% axis([0 25 0 25])
% axis equal

wp = [0.2 20 20 8 8 13.2; 3 3 22 22 16 16; 0 pi/2 pi 3*pi/2 2*pi 2*pi];

plot(wp(1,:), wp(2,:), 'x')

xlabel('X (m)')
ylabel('Y (m)')
xticks(0:1:25)
yticks(0:1:25)
grid on

% ======================================================
% 
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss);
sv.Map = map;

%%%%%%%%%%%%%%%
tic;
%%%%%%%%%%%%%%%

% 
planner = plannerHybridAStar(sv, ...
                             MinTurningRadius=4,...
                             NumMotionPrimitives=5,...
                             AnalyticExpansionInterval=5,...
                             MotionPrimitiveLength=6);


% 
startPose = [0.2,3.0,0];  % [meter, meter, rad]
goalPose = [13.2,16.0,0];

% 
refpath = plan(planner, startPose, goalPose); 

% 
show(map);
hold on;

%
show(planner)
hold on;

% 
plot(startPose(1), startPose(2), 'g*', 'MarkerSize', 10);
plot(goalPose(1), goalPose(2), 'b+', 'MarkerSize', 10);

xlabel('X (m)');
ylabel('Y (m)');
title('Hybrid A* Path Planning');
grid on;

elapsedTime = toc;
fprintf('A*Computation time: %.4f\n', elapsedTime);