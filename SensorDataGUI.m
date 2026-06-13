classdef SensorDataGUI < matlab.apps.AppBase

    properties (Access = private)
        UIFigure         matlab.ui.Figure
        TemperatureLabel matlab.ui.control.Label
        HumidityLabel    matlab.ui.control.Label
        MoistureLabel    matlab.ui.control.Label
        pHLabel          matlab.ui.control.Label
        ArduinoObj       % Arduino object
        AnalogPin        % Analog pin for reading sensor data
        RefreshRate      % Refresh rate for updating GUI (in seconds)
        TimerObj         % Timer object for updating GUI
    end

    methods (Access = private)

        % Function to initialize Arduino and GUI
        function initialize(app)
            % Create Arduino object
            app.ArduinoObj = arduino();

            % Define analog pin for each sensor
            app.AnalogPin = struct('Temperature', 'A0', ...
                                   'Humidity', 'A1', ...
                                   'Moisture', 'A2', ...
                                   'pH', 'A3');

            % Set refresh rate for updating GUI (in seconds)
            app.RefreshRate = 1;

            % Create timer object
            app.TimerObj = timer('ExecutionMode', 'fixedRate', ...
                                 'Period', app.RefreshRate, ...
                                 'TimerFcn', @(~, ~) updateData(app));

            % Start the timer
            start(app.TimerObj);
        end

        % Function to update sensor data and GUI labels
        function updateData(app)
            % Read sensor data from Arduino
            temperature = readVoltage(app.ArduinoObj, app.AnalogPin.Temperature);
            humidity = readVoltage(app.ArduinoObj, app.AnalogPin.Humidity);
            moisture = readVoltage(app.ArduinoObj, app.AnalogPin.Moisture);
            pH = readVoltage(app.ArduinoObj, app.AnalogPin.pH);

            % Update GUI labels
            app.TemperatureLabel.Text = sprintf('Temperature: %.2f V', temperature);
            app.HumidityLabel.Text = sprintf('Humidity: %.2f V', humidity);
            app.MoistureLabel.Text = sprintf('Moisture: %.2f V', moisture);
            app.pHLabel.Text = sprintf('pH: %.2f V', pH);
        end

        % Function to close the Arduino connection and stop the timer
        function closeConnection(app)
            % Stop the timer
            stop(app.TimerObj);

            % Delete the timer object
            delete(app.TimerObj);

            % Clear the Arduino object
            clear app.ArduinoObj;
        end

        % Code that executes after component creation
        function startupFcn(app)
            % Initialize Arduino and GUI
            initialize(app);
        end

        % Code that executes on app deletion
        function delete(app)
            % Close the Arduino connection and stop the timer
            closeConnection(app);
        end
    end

    % AppDesigner callbacks
    methods (Access = private)

        % Callback function for UIFigure CloseRequest event
        function UIFigureCloseRequest(app, ~)
            % Delete the app
            delete(app);
        end
    end

    % AppDesigner initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)
            % Create UIFigure
            app.UIFigure = uifigure('Name', 'Sensor Data GUI', ...
                'CloseRequestFcn', @(~,~) UIFigureCloseRequest(app));

            % Create TemperatureLabel
            app.TemperatureLabel = uilabel(app.UIFigure);
            app.TemperatureLabel.Position