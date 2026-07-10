function optimize_dlqr_g()
    % Имя вашей модели Simulink (без расширения .slx)
    model_name = 'asbQuadcopter'; 
    
    % Границы поиска для коэффициента G (сузили, чтобы дрон не падал)
    G_min   = 1.0;   
    G_max   = 6.0; 
    
    % Настройки оптимизатора
    options = optimset('Display', 'iter', 'TolX', 1e-4);
    
    % Загружаем модель в память
    load_system(model_name);
    
    fprintf('Запуск оптимизации коэффициента G через Simulink (интервал: [%.1f, %.1f])...\n', G_min, G_max);
    
    % Используем fminbnd для одномерной оптимизации в границах
    [G_best, fval] = fminbnd(@(g) objective_g(g, model_name), G_min, G_max, options);
    
    % --- ФИНАЛЬНЫЙ ВЫВОД РЕЗУЛЬТАТОВ (БЕЗОПАСНЫЙ) ---
    assignin('base', 'G_opt', G_best);
    
    try
        simOut = sim(model_name, 'SimulationMode', 'normal', 'SrcWorkspace', 'base');
        ref_data = simOut.ref_signal.Data(:)';
        out_data = simOut.out_signal.Data(:)';
        r_matrix = corrcoef(ref_data, out_data);
        final_corr = r_matrix(1,2);
    catch
        final_corr = NaN; % Если модель упала на лучшем значении
    end
    
    fprintf('\n=== Оптимизация завершена ===\n');
    fprintf('Оптимальное значение коэффициента G: %.6f\n', G_best);
    if ~isnan(final_corr)
        fprintf('Финальный коэффициент корреляции: %.4f\n', final_corr);
    else
        fprintf('Предупреждение: На лучшем G модель выдает аварийную остановку (Assertion).\n');
    end
    fprintf('Минимальное значение функции стоимости: %.4f\n', fval);
end

%% Функция критерия стоимости для G
function cost = objective_g(g, model_name)
    assignin('base', 'G_opt', g);
    
    try
        % Запуск симуляции
        simOut = sim(model_name, 'SimulationMode', 'normal', 'SrcWorkspace', 'base');
        
        % Извлечение сигналов из To Workspace
        ref_data = simOut.ref_signal.Data(:)'; 
        out_data = simOut.out_signal.Data(:)';
        
        % 1. Компонента корреляции
        r_matrix = corrcoef(ref_data, out_data);
        if any(isnan(r_matrix))
            cost = 2; 
            return;
        end
        corr_component = 1 - r_matrix(1, 2); 
        
        % 2. Компонента отслеживания амплитуды (MSE)
        mse_error = mean((ref_data - out_data).^2);
        mse_scale = 1e3; 
        scaled_mse = mse_error * mse_scale;
        
        % 3. Итоговый баланс
        cost = 0.9 * corr_component + 0.1 * scaled_mse;
        
    catch
        % Если дрон упал/столкнулся с землей (Assertion), 
        % возвращаем максимальный штраф, чтобы fminbnd больше не шел в эту сторону
        cost = 2; 
    end
end