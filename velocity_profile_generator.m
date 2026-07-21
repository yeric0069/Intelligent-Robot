function velocity_profile_generator(output_waypoints)
% velocity_profile_generator: generate velocity and acceleration profiles between waypoints
% Inputs:
%   output_waypoints - [Nx3] matrix, each row is [x, y, heading]

% Settings
v_max = 3.0;          % maximum velocity (m/s)
a_max = 1.5;          % maximum acceleration (m/s^2) (less aggressive)
d_max = 2.0;          % maximum deceleration (m/s^2) (less aggressive)

% Initialize storage
trajectory_speed = [];
trajectory_acc = [];
time_stamps = [];

current_time = 0;

for i = 1:(size(output_waypoints, 1)-1)
    % Extract waypoints
    p1 = output_waypoints(i, 1:2);
    p2 = output_waypoints(i+1, 1:2);

    % Calculate distance
    distance = norm(p2 - p1);

    % Phase 1: Acceleration to v_max
    t_acc = v_max / a_max;         % time to accelerate to v_max
    d_acc = 0.5 * a_max * t_acc^2; % distance covered during acceleration

    % Phase 3: Deceleration to 0
    t_dec = v_max / d_max;
    d_dec = 0.5 * d_max * t_dec^2;

    % Check if there is a cruising phase
    if distance > (d_acc + d_dec)
        % Normal case: accel -> cruise -> decel
        d_cruise = distance - (d_acc + d_dec);
        t_cruise = d_cruise / v_max;

        % Time vector
        t_segment = linspace(0, t_acc + t_cruise + t_dec, 100);
        v_segment = zeros(size(t_segment));
        a_segment = zeros(size(t_segment));

        for k = 1:length(t_segment)
            t = t_segment(k);
            if t <= t_acc
                v_segment(k) = a_max * t; % Acceleration phase
                a_segment(k) = a_max;
            elseif t <= (t_acc + t_cruise)
                v_segment(k) = v_max;     % Cruise phase
                a_segment(k) = 0;
            else
                v_segment(k) = v_max - d_max * (t - t_acc - t_cruise); % Deceleration phase
                a_segment(k) = -d_max;
            end
        end
    else
        % Special case: No cruising, just accelerate and immediately decelerate
        v_peak = sqrt((2 * a_max * d_max * distance) / (a_max + d_max));
        t_acc = v_peak / a_max;
        t_dec = v_peak / d_max;

        t_segment = linspace(0, t_acc + t_dec, 100);
        v_segment = zeros(size(t_segment));
        a_segment = zeros(size(t_segment));

        for k = 1:length(t_segment)
            t = t_segment(k);
            if t <= t_acc
                v_segment(k) = a_max * t;
                a_segment(k) = a_max;
            else
                v_segment(k) = v_peak - d_max * (t - t_acc);
                a_segment(k) = -d_max;
            end
        end
    end

    % Store into total trajectory
    trajectory_speed = [trajectory_speed, v_segment]; %#ok<AGROW>
    trajectory_acc = [trajectory_acc, a_segment]; %#ok<AGROW>
    time_stamps = [time_stamps, current_time + t_segment]; %#ok<AGROW>

    % Update current time
    current_time = time_stamps(end);
end

% Plot results
figure;
tiledlayout(2,1);

nexttile;
plot(time_stamps, trajectory_speed, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Velocity Profile between Waypoints');
grid on;

nexttile;
plot(time_stamps, trajectory_acc, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration Profile between Waypoints');
grid on;

end

