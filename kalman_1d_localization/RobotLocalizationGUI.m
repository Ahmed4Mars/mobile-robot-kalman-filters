function RobotLocalizationGUI()
    % RobotLocalizationGUI
    %
    % This function launches an interactive MATLAB GUI for simulating
    % and visualizing a 1D robot localization scenario.
    %
    % Features:
    %  - Top axis: Visualization of a 1D robot (rectangle with two wheels).
    %  - Middle axis: Dynamic Gaussian distribution representing the belief
    %    about the robot's position.
    %  - Right axis: History plot showing true, estimated, and measured positions.
    %  - Sliders to control:
    %       * True robot position.
    %       * Initial estimated state.
    %       * Initial uncertainty (variance).
    %  - Button to clear the position history plot.
    %  - Arrow key input:
    %       * Left/right arrows move the robot and update the filter.
    %       * Periodic noisy measurements update the Kalman filter.

    %% Create and configure the figure window
    hFig = figure('Name', '1D Robot Localization', ...
                  'NumberTitle', 'off', ...
                  'Units', 'normalized', ...
                  'WindowState', 'maximized', ...
                  'KeyPressFcn', @keyPressHandler);

    %% Create axes for different visualizations
    hAxTop     = axes('Parent', hFig, 'Units', 'normalized', ...
                      'Position', [0.05, 0.55, 0.55, 0.4]);
    hAxMiddle  = axes('Parent', hFig, 'Units', 'normalized', ...
                      'Position', [0.05, 0.1, 0.55, 0.3]);
    hAxHistory = axes('Parent', hFig, 'Units', 'normalized', ...
                      'Position', [0.65, 0.55, 0.3, 0.4]);

    %% Initial simulation parameters
    robotPos  = 5;            % True robot position
    robotVar  = 0.2;          % Initial uncertainty (variance)
    xRange    = 0:0.01:10;    % Range for plots
    dxStep    = 0.1;          % Movement step for keyboard input

    % Kalman filter initial estimate
    x_k = robotPos;           % Estimated state
    P_k = robotVar;           % Estimated uncertainty
    z_k = -1;                 % Measurement initialized to be N/A

    % Position tracking arrays
    posHistory = [robotPos];  % True position history
    xHistory   = [x_k];       % Estimated state history
    zHistory   = [z_k];       % Measurement history

    % Kalman filter parameters
    z_Ts = 2;
    var_u = 0.03;
    var_z = 1;          % Measurement noise variance

    %% Robot geometry specifications
    bodyWidth  = 0.4;
    bodyHeight = 0.1;
    wheelR     = 0.025;
    wheelD     = 2 * wheelR;
    bodyY      = wheelD;

    %% Top axis: Robot drawing
    axes(hAxTop);
    hold on; grid on;
    plot(xRange, zeros(size(xRange)), 'k-');

    hRect = rectangle('Position', [robotPos - bodyWidth/2, bodyY, bodyWidth, bodyHeight], ...
                      'FaceColor', [0.7 0.8 1.0], 'EdgeColor', 'k');
    hWheel1 = rectangle('Position', [robotPos - bodyWidth/2 - wheelR, 0, wheelD, wheelD], ...
                        'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3]);
    hWheel2 = rectangle('Position', [robotPos + bodyWidth/2 - wheelR, 0, wheelD, wheelD], ...
                        'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3]);

    xlim([0, 10]);
    ylim([-0.2, 0.5]);
    title('1D Mobile Robot Visualization');
    xlabel('Position');
    ylabel('Robot');

    %% Middle axis: Gaussian belief
    axes(hAxMiddle);
    hold on; grid on;
    yGauss = gaussFunction(xRange, x_k, P_k);
    hGauss = plot(xRange, yGauss, 'b-', 'LineWidth', 2);
    xlim([0, 10]);
    ylim([0, 1.2 * max(yGauss)]);
    title('Gaussian Distribution bel(x_k)');
    xlabel('x');
    ylabel('Probability Density');

    %% Right axis: History plot
    axes(hAxHistory);
    hold on; grid on;
    hHistory_ax = plot(1:length(posHistory), posHistory, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    xHistory_ax = plot(1:length(xHistory), xHistory, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    zHistory_ax = plot(nan, nan, 'go', 'LineWidth', 1.5, 'MarkerSize', 6);

    title('Robot Position History');
    xlabel('Step Index');
    ylabel('Position');
    legend('true', 'estimated', 'measured');
    xlim([0, 10]);
    ylim([0, 10]);

    %% Sliders and controls
    uicontrol('Style', 'text', 'Parent', hFig, 'Units', 'normalized', ...
              'String', 'Robot Position', 'HorizontalAlignment', 'left', ...
              'Position', [0.70, 0.38, 0.2, 0.03]);

    hSliderPos = uicontrol('Style', 'slider', 'Parent', hFig, 'Units', 'normalized', ...
                           'Min', 0, 'Max', 10, 'Value', robotPos, ...
                           'Position', [0.70, 0.35, 0.2, 0.03], ...
                           'Callback', @updateFromSlider);

    uicontrol('Style', 'text', 'Parent', hFig, 'Units', 'normalized', ...
              'String', 'Initial state', 'HorizontalAlignment', 'left', ...
              'Position', [0.70, 0.30, 0.2, 0.03]);

    hSliderState = uicontrol('Style', 'slider', 'Parent', hFig, 'Units', 'normalized', ...
                             'Min', 0, 'Max', 10, 'Value', x_k, ...
                             'Position', [0.70, 0.27, 0.2, 0.03], ...
                             'Callback', @updateFromSlider);

    uicontrol('Style', 'text', 'Parent', hFig, 'Units', 'normalized', ...
              'String', 'Initial Variance', 'HorizontalAlignment', 'left', ...
              'Position', [0.70, 0.22, 0.2, 0.03]);

    hSliderVar = uicontrol('Style', 'slider', 'Parent', hFig, 'Units', 'normalized', ...
                           'Min', 0.0001, 'Max', 2, 'Value', P_k, ...
                           'Position', [0.70, 0.19, 0.2, 0.03], ...
                           'Callback', @updateFromSlider);

    uicontrol('Style', 'pushbutton', 'Parent', hFig, 'Units', 'normalized', ...
              'String', 'Clear History', ...
              'Position', [0.70, 0.14, 0.2, 0.04], ...
              'Callback', @clearHistory);

    %% Slider callback
    function updateFromSlider(~, ~)
        robotPos = get(hSliderPos, 'Value');
        x_k = get(hSliderState, 'Value');
        P_k = get(hSliderVar, 'Value');

        % reset visible history so screenshot matches the chosen initial condition
        posHistory = [robotPos];
        xHistory   = [x_k];
        zHistory   = [-1];

        updateVisuals();
    end

    %% Keyboard callback
    function keyPressHandler(~, event)
        switch event.Key
            case 'leftarrow'
                u_k = -dxStep;
                robotPos = max(0, robotPos + u_k);

            case 'rightarrow'
                u_k = dxStep;
                robotPos = min(10, robotPos + u_k);

            otherwise
                return;
        end

        [x_k, P_k] = kalman_predict(x_k, P_k, u_k, 1, 1, var_u);

        % Add measurement periodically
        if mod(length(posHistory), z_Ts) == 0
            z_k = normrnd(robotPos, sqrt(var_z));
            [x_k, P_k, ~] = kalman_update(x_k, P_k, z_k, 1, var_z);
            zHistory(end+1) = z_k;
        else
            zHistory(end+1) = -1;
        end

        % Update history
        posHistory(end+1) = robotPos;
        xHistory(end+1)   = x_k;

        set(hSliderPos, 'Value', robotPos);
        updateVisuals();
    end

    %% Update visuals
    function updateVisuals()
        set(hRect, 'Position', [robotPos - bodyWidth/2, bodyY, bodyWidth, bodyHeight]);
        set(hWheel1, 'Position', [robotPos - bodyWidth/2 - wheelR, 0, wheelD, wheelD]);
        set(hWheel2, 'Position', [robotPos + bodyWidth/2 - wheelR, 0, wheelD, wheelD]);

        yGauss = gaussFunction(xRange, x_k, max(P_k, 0.0001));
        set(hGauss, 'YData', yGauss);
        ylim(hAxMiddle, [0, 1.2 * max(yGauss)]);

        set(hHistory_ax, 'XData', 1:length(posHistory), 'YData', posHistory);
        set(xHistory_ax, 'XData', 1:length(xHistory), 'YData', xHistory);

        validZ = zHistory >= 0;
        set(zHistory_ax, 'XData', find(validZ), 'YData', zHistory(validZ));

        xlim(hAxHistory, [1, max(10, length(posHistory))]);
        ylim(hAxHistory, [0, 10]);

        drawnow;
    end

    %% Clear history
    function clearHistory(~, ~)
        posHistory = [robotPos];
        xHistory   = [x_k];
        zHistory   = [-1];
        updateVisuals();
    end

    %% Gaussian helper
    function g = gaussFunction(x, mu, sigma2)
        sigma2 = max(sigma2, 0.0001);
        g = (1 / sqrt(2*pi*sigma2)) * exp(-0.5 * ((x - mu).^2 / sigma2));
    end

end
