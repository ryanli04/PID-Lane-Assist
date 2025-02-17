function visualizer(lateral_position, sim_time)
    persistent fig ax car road last_time car_width car_length

    % Ensure both inputs are provided
    if nargin < 2
        error('visualizer requires two inputs: lateral_position and sim_time');
    end

    % Initialize figure on first run
    if isempty(fig) || ~isvalid(fig)
        fig = figure('Name', 'Lane Keeping Visualization', ...
                     'NumberTitle', 'off', ...
                     'Position', [100, 100, 800, 600]);
        ax = axes(fig);
        hold(ax, 'on');
        axis(ax, [-5 5 -10 10]);
        grid(ax, 'on');

        % Define car dimensions
        car_width = 2;
        car_length = 4;

        % Draw road
        road_width = 3.5;
        plot(ax, [-road_width/2 -road_width/2], [-10 10], 'k-', 'LineWidth', 2);
        plot(ax, [road_width/2 road_width/2], [-10 10], 'k-', 'LineWidth', 2);
        plot(ax, [0 0], [-10 10], 'k--');

        % Create car
        car = rectangle(ax, 'Position', ...
            [lateral_position - car_width/2, -car_length/2, car_width, car_length], ...
            'FaceColor', [0.3 0.6 1], ...
            'EdgeColor', 'black', ...
            'LineWidth', 1.5);
        
        last_time = sim_time;
    end

    % Prevent too frequent updates
    if ~isempty(last_time) && (sim_time - last_time < 0.05)
        return;
    end
    last_time = sim_time;

    % Smoothly move the car (Fix for variable scope issues)
    start_x = car.Position(1);
    end_x = lateral_position - car_width / 2;

    for step = linspace(start_x, end_x, 10)
        car.Position(1) = step;
        title(ax, sprintf('Position: %.2f m | Time: %.2f s', lateral_position, sim_time));
        drawnow;
    end
end