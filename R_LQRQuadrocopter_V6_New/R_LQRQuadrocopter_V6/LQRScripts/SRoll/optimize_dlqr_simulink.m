function optimize_dlqr_simulink()
    % Имя вашей модели Simulink (без расширения .slx)
    model_name = 'asbQuadcopter'; 
    
    % 1. Исходные неизменные параметры (задаем в Workspace для Simulink)
    A = [1, 0.005, 0;
         0, 1, 0.005;
         0, 0, 0];
    B = [0; 0; 1.30767676];
    R = 1;
    
    assignin('base', 'A', A);
    assignin('base', 'B', B);
    assignin('base', 'R_opt', R);
    
    % 2. Начальное приближение диагонали матрицы Q [q1, q2, q3]
    q0 = [0.0015, 0.0001, 0.001]; 
    
    % Настройки оптимизатора
    options = optimset('Display', 'iter', ...
                       'MaxIter', 150, ...   % Ограничение итераций для экономии времени симуляции
                       'TolX', 1e-5);
    
    % Открываем модель в памяти (без визуального открытия окна)
    load_system(model_name);
    
    fprintf('Запуск многокритериальной оптимизации через Simulink...\n');
    
    % Запуск fminsearch
    [q_best, fval] = fminsearch(@(q) objective_simulink(q, model_name), q0, options);
    
   % ... (код fminsearch выше остаётся без изменений) ...
    
    % 3. Вывод результатов
    q_best = abs(q_best) + 1e-8; % Исключаем нули и отрицательные значения
    
    % !!! ИСПРАВЛЕНИЕ: Гарантированно записываем финальное Q в Base Workspace
    assignin('base', 'Q_opt', diag(q_best));
    
    % Запуск финальной симуляции с явным указанием использовать Base Workspace
    simOut = sim(model_name, 'SimulationMode', 'normal', 'SrcWorkspace', 'base');
    
    ref_data = simOut.ref_signal.Data(:)';
    out_data = simOut.out_signal.Data(:)';
    r_matrix = corrcoef(ref_data, out_data);
    final_corr = r_matrix(1,2);
    final_mse  = mean((ref_data - out_data).^2);
    
    fprintf('\n=== Оптимизация успешно завершена ===\n');
    fprintf('Оптимальные коэффициенты диагонали Q: [%.8f, %.8f, %.8f]\n', q_best);
    fprintf('Финальный коэффициент корреляции: %.4f\n', final_corr);
    fprintf('Финальная среднеквадратичная ошибка (MSE): %e\n', final_mse);
    fprintf('Значение функции стоимости: %.4f\n', fval);
end

%% Функция критерия оптимальности (Очищенная версия)
function cost = objective_simulink(q, model_name)
    % Веса Q должны быть строго положительными
    q = abs(q) + 1e-8; 
    Q_sim = diag(q);
    R_sim = 1; 
    
    % Записываем переменные ТОЛЬКО в Base Workspace
    assignin('base', 'Q_opt', Q_sim);
    assignin('base', 'R_opt', R_sim);
    
    try
        % Запуск симуляции с жестким требованием брать переменные из Base
        simOut = sim(model_name, 'SimulationMode', 'normal', 'SrcWorkspace', 'base');
        
        % Извлечение данных из блоков To Workspace
        ref_data = simOut.ref_signal.Data(:)'; 
        out_data = simOut.out_signal.Data(:)';
        
        % 1. Расчет компоненты корреляции
        r_matrix = corrcoef(ref_data, out_data);
        if any(isnan(r_matrix))
            cost = 2; 
            return;
        end
        corr_component = 1 - r_matrix(1, 2);
        
        % 2. Расчет компоненты отслеживания (MSE)
        mse_error = mean((ref_data - out_data).^2);
        mse_scale = 1e6; 
        scaled_mse = mse_error * mse_scale;
        
        % 3. Компромиссная функция стоимости
        cost = 0.9 * corr_component + 0.1 * scaled_mse;
        
    catch ME
        % Если упало — выводим детальную ошибку, чтобы понять, какой блок внутри FCS сбоит
        fprintf('Ошибка симуляции: %s\n', ME.message);
        cost = 2; 
    end
end