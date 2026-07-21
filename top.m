clear; close all; clc;

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

inflate(map, 1.1);

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

% modified waypoints
waypoints = [0.2,  3.0,  0;      
            20.0, 3.0,  pi/2;    
            20.0, 22.0, pi/2;      
            13.8,  24.0, pi;
            8.0,  22.0, 3*pi/2;
            11.0,  16.0, 2*pi;    
            13.2, 16.0, 2*pi];     

% %%origin waypoints
% waypoints = [0.2,  3.0,  0;      
%              20.0, 3.0,  pi/2;   
%              20.0, 22.0, pi;     
%              8.0,  22.0, 3*pi/2; 
%              8.0,  16.0, 2*pi;   
%              13.2, 16.0, 2*pi];    

plot(waypoints(:,1), waypoints(:,2), 'x')

xlabel('X (m)')
ylabel('Y (m)')
xticks(0:1:25)
yticks(0:1:25)
grid on

%radii   = [3; 3; 3; 3];     % Radii of obstacle, 3 for all radii
%centres = [3.2 14.0; 3.2 20.0; 14.2 8.0; 14.2 20.0];   % Center of obstacle
 radii   = [3; 3; 3; 3; 3];     % Radii of obstacle, 3 for all radii
 centres = [3.2 14.0; 3.2 20.0; 14.2 8.0; 14.2, 20.0; 22.7 10.0];   % Center of obstacle

cnt_num = 1;
ptr = 1;

% start_point = waypoints(1, 1:3);
start_point = waypoints(cnt_num, 1:3);
% for idx = 1:size(waypoints,1)-1

%%%%%%%%%%%%%%%%%%
% Hybird A* Path
%%%%%%%%%%%%%%%%%%

for idx = cnt_num:size(waypoints,1)-1
    %refpath = RRT_planner(start_point, waypoints(idx+1,1:3), map);
    refpath = hybrid_A(start_point, waypoints(idx+1,1:3), map);
    segment = refpath.States;
    
    output_waypoints(ptr:ptr+size(segment,1)-2,:) = segment(1:end-1,:);
    ptr = ptr + size(segment,1) - 1;

    start_point = segment(end,:);
end

output_waypoints(ptr,:) = start_point;

%% plot 
plot(output_waypoints(:, 1), output_waypoints(:, 2), 'r-', 'LineWidth', 1);
viscircles(centres, radii);
xlabel('X (m)');
ylabel('Y (m)');
title('Path Planning');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% simple waypoints trajectory
plot(waypoints(:,1), waypoints(:,2), 'g--', 'LineWidth', 2); % comment this out for part2_1, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ini_state = [waypoints(cnt_num,:), 0, 0]';
h_mpc = animatedline('Color', 'c', 'LineWidth', 3);
h_current_pos = plot(ini_state(1), ini_state(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

all_xresult = [];
all_uresult = [];
ptr_x = 1;
ptr_u = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%
% RHP Trajectory
%%%%%%%%%%%%%%%%

for idx = 1:size(output_waypoints,1)-1
    [xresult, ~, uresult] = finite_horizon_solver(ini_state, output_waypoints(idx+1,1:3)', radii, centres);
    
    % draw driving curves
    for k = 1:size(xresult, 2)
        addpoints(h_mpc, xresult(1,k), xresult(2,k));
        set(h_current_pos, 'XData', xresult(1,k), 'YData', xresult(2,k));
        drawnow limitrate; % refresh curve immediately
    end
    
    all_xresult(:, ptr_x:ptr_x+size(xresult,2)-2) = xresult(:, 1:end-1);
    ptr_x = ptr_x + size(xresult,2) - 1;

    all_uresult(:, ptr_u:ptr_u+size(uresult,2)-1) = uresult(:, 1:end);
    ptr_u = ptr_u + size(uresult,2);

    ini_state = xresult(:, end); % update initial state
end

drawnow; % draw all curves

all_xresult(:, end+1) = xresult(:, end);
all_uresult(:, end+1) = uresult(:, end);

result_plot(all_xresult, all_uresult)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elapsedTime = toc;
fprintf('Computation time: %.4f\n', elapsedTime);
