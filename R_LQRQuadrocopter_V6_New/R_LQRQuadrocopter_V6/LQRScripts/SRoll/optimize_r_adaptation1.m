function optimize_r_adaptation()
    clc; % Очищаем только экран консоли, память НЕ ТРОГАЕМ, чтобы не стереть параметры БПЛА
    
    model_name = 'flightController'; 
    
    % Удаляем ТОЛЬКО переменную оптимизации, если она была, не задевая параметры модели
    if evalin('base', 'exist(''R_ADAPT_PARAMS'',''var'')')
        evalin('base', 'clear R_ADAPT_PARAMS');
    end
    
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end
    
    % Стартовый вектор (8 параметров)
    x0 = [1e-5, 4e-6, 2e-4, 1e-3, 0.5, 0.001, 0.001, 0.0009];
    
    % Границы поиска
    lb = [1e-7, 1e-7, 1e-6, 1e-5, 0.001, 0.0001, 1e-7,  1e-8]; 
    ub = [1e-3, 1e-3, 1e-2, 1e-1, 1.0,   0.5,    0.1,   0.095]; 
    
    options = optimset('Display', 'iter', ...
                       'MaxIter', 150, ... % Ограничим для теста
                       'TolX', 1e-5, ...
                       'TolFun', 1e-5);
                   
    fprintf('=== СТАРТ ОПТИМИЗАЦИИ ДИНАМИКИ R-АДАПТАЦИИ ===\n');
    
    % Пробный запуск для проверки работоспособности модели перед оптимизацией
    fprintf('Тестовый запуск модели... ');
    test_J = objective_func(x0, model_name, lb, ub);
    if test_J >= 1e10
        warning('ВНИМАНИЕ: Модель не смогла запуститься даже на стартовых параметрах! См. ошибку выше.');
        return;
    else
        fprintf('Успешно! Начальное значение J = %.4f\n\n', test_J);
    end
    
    % Запуск fminsearch
    [x_opt, fval] = fminsearch(@(x) objective_func(x, model_name, lb, ub), x0, options);
    
    assignin('base', 'R_ADAPT_PARAMS', x_opt);
    
    fprintf('\n=== ОПТИМАЛЬНЫЕ ПАРАМЕТРЫ АДАПТАЦИИ ===\n');
    fprintf('e_on:              %.8f\n', x_opt(1));
    fprintf('e_off:             %.8f\n', x_opt(2));
    fprintf('R0 (базовый):      %.6f\n', x_opt(7));
    fprintf('Rspan (диапазон):  %.6f\n', x_opt(8));
    fprintf('Итоговый критерий качества J: %.6f\n', fval);
end

function J = objective_func(x, model_name, lb, ub)
    % Проверка границ
    if any(x < lb) || any(x > ub) || x(2) >= x(1) || x(8) >= x(7)
        J = 1e12; 
        return;
    end
    
    assignin('base', 'R_ADAPT_PARAMS', x);
    
    % Запуск симуляции с перехватом и выводом ошибок в консоль
    try
        simOut = sim(model_name, ...
                     'SrcWorkspace', 'base', ... 
                     'ReturnWorkspaceOutputs', 'on');
    catch ME
        % Если симуляция упала — выводим сообщение об ошибке прямо в консоль!
        fprintf('\n[ОШИБКА СИМУЛЯЦИИ]: %s\n', ME.message);
        if ~isempty(ME.cause)
            for k = 1:length(ME.cause)
                fprintf('  Причина: %s\n', ME.cause{k}.message);
            end
        end
        J = 1e10; 
        return;
    end
    
    % Извлечение данных
    e_sim = []; u_sim = [];
    try e_sim = simOut.get('e'); catch, end
    try u_sim = simOut.get('u'); catch, end

    if isempty(e_sim) && evalin('base', 'exist(''e'',''var'')')
        e_sim = evalin('base', 'e');
    end
    if isempty(u_sim) && evalin('base', 'exist(''u'',''var'')')
        u_sim = evalin('base', 'u');
    end

    if isempty(e_sim) || isempty(u_sim)
        fprintf('\n[ОШИБКА]: Данные "e" или "u" не найдены в результатах симуляции!\n');
        J = 1e12; 
        return;
    end
    
    % Приведение типов данных
    if isa(e_sim, 'timeseries'), e = e_sim.Data;
    elseif isstruct(e_sim), e = e_sim.signals.values;
    else, e = e_sim; end

    if isa(u_sim, 'timeseries'), u = u_sim.Data;
    elseif isstruct(u_sim), u = u_sim.signals.values;
    else, u = u_sim; end

    e = double(e(:));
    u = double(u(:));
    
    % Расчет критерия J
    Ts = 0.005; 
    N = length(e);
    t_vector = (0:N-1)' * Ts; 
    
    MAE      = mean(abs(e));
    RMSE     = sqrt(mean(e.^2));
    MaxError = max(abs(e));
    
    IAE      = trapz(abs(e)) * Ts;
    ISE      = trapz(e.^2) * Ts;
    ITAE     = trapz(t_vector .* abs(e)) * Ts;
    TV_error = sum(abs(diff(e))); 
    
    TV_control = sum(abs(diff(u))); 
    U_RMS      = sqrt(mean(u.^2));
    
    w_accuracy = 100.0; 
    w_control  = 0.5;   
    
    J = w_accuracy * (MAE + RMSE + MaxError*0.1 + IAE + ISE*10 + ITAE*0.1 + 0.05*TV_error) + ...
        w_control * (U_RMS*0.1 + TV_control * 3.0);
end