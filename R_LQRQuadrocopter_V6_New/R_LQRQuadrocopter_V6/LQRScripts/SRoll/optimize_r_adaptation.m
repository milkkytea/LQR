function optimize_r_adaptation()
    clc; 
    warning('off', 'all'); % Полностью выключает вывод варнингов в консоль
    model_name = 'asbQuadcopter'; 
    
    % Загружаем систему, если она еще не открыта
    if ~bdIsLoaded(model_name)
        load_system(asbQuadcopter);
    end
    
    % ГАРАНТИРОВАННО ОТКЛЮЧАЕМ Fast Restart во избежание кэширования параметров!
    set_param('asbQuadcopter', 'FastRestart', 'off');
    
    % Начальный вектор параметров (8 значений) - пусть он будет заведомо стабильным
    % Сделали R0 = 1 (было 0.01) и Rspan = 0.9.
    % ВНИМАНИЕ: Проверьте соответствие границ ниже!
    x0 = [1e-5, 4e-6, 2e-4, 1e-3, 0.5, 0.001, 0.1, 0.01];
    
    % Векторы ограничений (должны строго соответствовать размерности x0)
    % Если x0(7) = 1, то верхняя граница ub(7) должна быть больше или равна 1 (сейчас там 0.05).
    % Давайте скорректируем ub(7) и ub(8), чтобы x0 не вылетал за границы при первом же шаге!
    lb = [1e-6, 1e-6, 1e-5, 1e-4, 0.01,  0.0001, 0.0005, 0.0001]; 
    ub = [1e-3, 1e-3, 1e-2, 1e-1, 1.0,   0.5,    2.0,    1.5]; % Увеличили верхние границы для R0 и Rspan
    
    options = optimset('Display', 'iter', ...
                       'MaxIter', 150, ... 
                       'TolX', 1e-5, ...
                       'TolFun', 1e-5);
                   
    fprintf('=== СТАРТ ОПТИМИЗАЦИИ ДИНАМИКИ R-АДАПТАЦИИ ===\n');
    
    % Пробный запуск
    fprintf('Тестовый запуск модели... ');
    test_J = objective_func(x0, model_name, lb, ub);
    if test_J >= 1e10
        warning('ВНИМАНИЕ: Ошибка инициализации модели!');
        return;
    else
        fprintf('Успешно! Начальное значение J = %.4f\n\n', test_J);
    end
    
    % Запуск процесса оптимизации fminsearch
    [x_opt, fval] = fminsearch(@(x) objective_func(x, model_name, lb, ub), x0, options);
    
    % Записываем финальный оптимальный вектор в Workspace
    assignin('base', 'R_ADAPT_PARAMS', x_opt);
    
    fprintf('\n=== ОПТИМАЛЬНЫЕ ПАРАМЕТРЫ АДАПТАЦИИ ===\n');
    fprintf('e_on:              %.8f\n', x_opt(1));
    fprintf('e_off:             %.8f\n', x_opt(2));
    fprintf('R0 (базовый):      %.6f\n', x_opt(7));
    fprintf('Rspan (диапазон):  %.6f\n', x_opt(8));
    fprintf('Итоговый критерий качества J: %.6f\n', fval);
    
    warning('on', 'all'); % Возвращаем варнинги на место
end