clc; clear; close all;

% Base PID gains
Kp_base = 9.0;
Ki_base = 3;
Kd_base = 8;

% Vehicle parameters
L = 2.5;  % Wheelbase (meters)
dt = 0.01;  % Time step
t_end = 60;  % Simulation time
t = 0:dt:t_end;  % Time vector

% Steering constraints
max_steering_angle = 0.4;  % Max ± angle (radians)
max_steering_rate = 0.2;  % Max change in angle per step

% Test scenario: Vehicle starts with lateral offset
speeds = [10, 15, 25, 30];  % Expanded speed range
initial_offset = -1;  % Initial lateral deviation (meters)

% Road Pattern
road_length = 1000;
x_waypoints = linspace(0, road_length, 10);
y_waypoints = sin(x_waypoints / 10) * 2.5;
spline_curve = spline(x_waypoints, y_waypoints);

% State-space matrices (realistic dynamics)
A = [0 1; 0 -0.05];
B = [0; 1];
C = [1 0];

figure;
for j = 1:length(speeds)
    v = speeds(j);
    
    % Adjust PID gains smoothly based on speed
    speed_factor = v / 50;  % Normalize speed factor (0 to 1)
    Kp = Kp_base * (1 + 0.09 * v);  % Moderate scaling
    Kd = Kd_base * (1 + 0.08 * v);  % Increase damping at high speeds
    Ki = Ki_base * (1 + 0.01 * v);  % Low integral action to avoid windup
    
    % Initialize state
    x = zeros(2, length(t));
    x(:,1) = [initial_offset; 0];
    
    error = zeros(size(t));
    u = zeros(size(t));
    integral_error = 0;
    prev_error = 0;
    prev_u = 0;
    
    noise_level = 0.001;
    
    for i = 2:length(t)
        x_position = v * t(i);
        x_position = max(min(x_position, max(x_waypoints)), min(x_waypoints));
        desired_y = ppval(spline_curve, x_position);
        noisy_lateral_position = x(1,i-1) + noise_level * randn;
        
        % Compute error
        error(i) = desired_y - noisy_lateral_position;
        integral_error = integral_error + error(i) * dt;
        integral_error = max(min(integral_error, 0.8 + 0.02 * v), -0.8 - 0.02 * v);  % Reduced integral clamping
        
        derivative = (error(i) - prev_error) / dt;
        prev_error = error(i);
        
        % PID control
        raw_steering = Kp * error(i) + Ki * integral_error + Kd * derivative;
        u(i) = tanh(raw_steering / 3);  % Scaling to avoid excessive corrections
        
        % Apply steering rate limit
        delta_u = max(min(u(i), prev_u + max_steering_rate), prev_u - max_steering_rate);
        prev_u = delta_u;
        u(i) = max(min(delta_u, max_steering_angle), -max_steering_angle);
        
        % State update
        dx = A * x(:,i-1) + B * u(i);
        x(:,i) = x(:,i-1) + dx * dt;
    end
    
    % Plot results
    subplot(length(speeds), 1, j);
    distances = v * t;
    plot(distances, x(1,:), 'b', 'LineWidth', 2);
    hold on;
    
    x_road = 0:1:min(max(distances), road_length);
    y_road = ppval(spline_curve, x_road);
    plot(x_road, y_road, 'r--', 'LineWidth', 2);
    
    yline(0, 'k--', 'LineWidth', 1);
    grid on;
    title(['Lateral Position vs. Distance (Speed = ' num2str(v) ' m/s)']);
    xlabel('Distance (m)');
    ylabel('Lateral Position (m)');
    legend('Vehicle Path', 'Desired Path');
    xlim([0 500]);
    ylim([-4 4]);
end

sgtitle('Lane Assistant with Improved PID Gain Scheduling for Different Speeds');
