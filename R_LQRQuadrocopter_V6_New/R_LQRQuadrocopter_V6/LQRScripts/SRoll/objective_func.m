function J = objective_func(x, model_name, lb, ub)
    % 1. Проверка ограничений 
    if any(x < lb) || any(x > ub) || x(2) >= x(1) 
        J = 1e12; 
        return;
    end
    
    % 2. СОЗДАЕМ ОБЪЕКТ НАСТРОЙКИ СИМУЛЯЦИИ
    simIn = Simulink.SimulationInput(model_name);
    
    % Принудительно записываем наш вектор параметров в пространство модели
    simIn = simIn.setVariable('R_ADAPT_PARAMS', double(x), 'Workspace', 'base');
    
 % 3. ЗАПУСКАЕМ СИМУЛЯЦИЮ С НОВЫМИ ПАРАМЕТРАМИ
    try
        simOut = sim(simIn); 
                 
        % 4. Извлекаем результаты
        t = simOut.get('tout');
        e = simOut.get('error'); 
        u = simOut.get('control_signal');
        
    catch ME
        % Если дрон врезался в землю (Assertion) или симуляция упала,
        % мы просто даем штраф 1e6 и идем дальше!
        J = 1e6;
        return;
    end
    % Расчет функционала качества J
    Ts = 0.005; 
    N = length(e);
    
    % === МАТЕМАТИЧЕСКИЕ СЛАГАЕМЫЕ ДЛЯ КРИТЕРИЯ КАЧЕСТВА ===
    MAE        = mean(abs(e));         % Средняя ошибка
    TV_error   = sum(abs(diff(e)));    % Колебательность самой ошибки
    
    % Показатели шума управления (вибрации моторов)
    TV_control = sum(abs(diff(u)));    % Полная вариация управления
    U_RMS      = sqrt(mean(u.^2));     % Энергия управления
    
    % === НАСТРОЙКА ВЕСОВЫХ КОЭФФИЦИЕНТОВ ДЛЯ МИНИМИЗАЦИИ ШУМА ===
    w_accuracy   = 5.0;     
    w_smoothness = 150.0;   
    w_energy     = 1.0;     
    
    % Итоговый критерий качества
    J = w_accuracy * (MAE + 0.1 * TV_error) + ...
        w_smoothness * TV_control + ...
        w_energy * U_RMS;
    
    % Лог в консоль для контроля изменения J
    fprintf('e_on: %.6f | R0: %.6f | Rsp: %.6f | J = %.4f\n', ...
            x(1), x(7), x(8), J);
end