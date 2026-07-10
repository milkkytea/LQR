

scenarios = {'Hover'; 'LandingSearch'};
     TFinals = [10; 15];
     in = Simulink.SimulationInput.empty(size(scenarios, 1), 0);
     for i = 1 : size(scenarios, 1)
     in(i) = Simulink.SimulationInput('asbQuadcopter');
     in(i) = in(i).setBlockParameter(['flightControlSystem/Flight Control System/' ...
     'landing logic/Position//Attitude Reference'], ...
     'ActiveScenario', scenarios{i});
     in(i) = in(i).setVariable('TFinal', TFinals(i), 'Workspace', 'asbQuadcopter');
     end
    
     out = sim(in(2)); % simulate with the "Hover" scenario for 10 seconds
     t = out.posref.time;
     xyzrpy = out.xyzrpy;
     estim = out.estim.signals.values;
     posref = out.posref.signals.values;
     motor = out.motor.signals.values;
     sensor = out.sensor.signals.values;

     plot(t, xyzrpy(:, 4), t, estim(:, 6), t, posref(:, 8), t, posref(:, 6), 'LineWidth', 2);
     legend('True roll', 'Estimated roll', 'Commanded roll', 'Reference roll', ...
    'Location', 'best');
     xlabel('Time [s]');
     ylabel('Roll [rad]');
     title('asbQuadcopter');
     grid on;

    plot(t, xyzrpy(:, 5), t, estim(:, 5), t, posref(:, 7), t, posref(:, 5), 'LineWidth', 2);
    legend('True pitch', 'Estimated pitch', 'Commanded pitch', 'Reference pitch', ...
    'Location', 'best');
    xlabel('Time [s]');
    ylabel('Pitch [rad]');
    title('asbQuadcopter');
    grid on;

