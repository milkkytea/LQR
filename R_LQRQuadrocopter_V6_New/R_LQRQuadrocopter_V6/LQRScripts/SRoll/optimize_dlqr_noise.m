function optimize_dlqr_noise()
    % Имя вашей модели Simulink
    model_name = 'asbQuadcopter'; 
    
    % 1. СТРОГИЕ БЕЗОПАСНЫЕ ГРАНИЦЫ для диагонали Q [q1, q2, q3]
    q_min = [0.01,   0.0001, 0.0001]; % Минимально допустимые значения
    q_max = [1.0,    0.05,   0.02];   % Максимально допустимые значения
    
    % Номинальная (заведомо рабочая) точка
    q0 = [0.1792, 0.0055, 0.0028]; 
    
    % Переводим физические параметры в безразмерное пространство z (от -pi/2 до pi/2)
    norm_q = (q0 - q_min) ./ (q_max - q_min);
    z0 = asin(2 * norm_q - 1); 
    
    % Записываем стартовые значения в Base Workspace
    assignin('base', 'Q_opt', diag(q0));
    assignin('base', 'R_opt', 1);
    
    % Настройки оптимизатора fminsearch
    options = optimset('Display', 'iter', ...
                       'MaxIter', 80, ...   
                       'TolX', 1e-4);
    
    % Загружаем модель в память
    if ~bdIsLoaded(model_name)
        load_system(model_name);
    end
    
    fprintf('Запуск БЕЗОПАСНОЙ оптимизации Q в жестких границах...\n');
    
    % Запуск оптимизации по вектору z
    [z_best, fval] = fminsearch(@(z) objective_noise_bounded(z, q_min, q_max, model_name), z0, options);
    
    % --- ДЕКОДИРОВАНИЕ И ФИНАЛЬНЫЙ ВЫВОД РЕЗУЛЬТАТОВ ---
    q_best = q_min + (q_max - q_min) .* (sin(z_best) + 1) / 2;
    Q_final = diag(q_best);
    
    % Сохраняем лучшие коэффициенты в Base Workspace
    assignin('base', 'Q_opt', Q_final);
    assignin('base', 'R_opt', 1);
    
    fprintf('\n=== Оптимизация завершена ===\n');
    fprintf('Оптимальные коэффициенты диагонали Q: [%.8f, %.8f, %.8f]\n', q_best(1), q_best(2), q_best(3));
    fprintf('Финальное значение функции стоимости: %.4f\n', fval);
    
    try
        % НАСТРОЙКА СИМУЛЯЦИИ НА BASE WORKSPACE (КРИТИЧЕСКИ ВАЖНО)
        simOpts = simset('SrcWorkspace', 'base');
        simOut = sim(model_name, [], simOpts);
        
        metrics = calculate_metrics(simOut);
        fprintf('\n--- Финальные метрики системы ---\n');
        disp(metrics);
    catch ME
        fprintf('\n[Предупреждение] Финальный прогон завершился сбоем.\n');
        fprintf('Детали ошибки: %s\n', ME.message);
    end
end 

%% Функция критерия стоимости с автоматическим ограничением параметров
function cost = objective_noise_bounded(z, q_min, q_max, model_name)
    % Преобразуем виртуальный вектор z в строго ограниченные физические параметры q
    q = q_min + (q_max - q_min) .* (sin(z) + 1) / 2;
    
    Q_sim = diag(q);
    R_sim = 1; 
    
    % Обновляем переменные в Base Workspace, чтобы их подхватила симуляция
    assignin('base', 'Q_opt', Q_sim);
    assignin('base', 'R_opt', R_sim);
    
    try
        % Запуск симуляции с чтением параметров из Base Workspace
        simOpts = simset('SrcWorkspace', 'base');
        simOut = sim(model_name, [], simOpts);
        
        % Расчет метрик качества управления и уровня шума
        metrics = calculate_metrics(simOut);
        
        HF_RMS = metrics.Value(strcmp(metrics.Metric, 'HF_RMS'));
        r_cor  = metrics.Value(strcmp(metrics.Metric, 'r_cor'));
        
        if isnan(HF_RMS)
            HF_RMS = metrics.Value(strcmp(metrics.Metric, 'StdError'));
        end
        
        if isnan(r_cor) || r_cor < 0
            cost = 10;
            return;
        end
        
        % КОМПРОМИССНЫЙ КРИТЕРИЙ: 80% вес на ВЧ-шум, 20% вес на качество слежения
        cost = 0.8 * (HF_RMS * 100) + 0.2 * (1 - r_cor);
        
    catch
        % Возвращаем штрафной балл при ошибке симуляции
        cost = 10; 
    end
end 

%% Вспомогательная функция расчета метрик
function metrics = calculate_metrics(simOut)
    t1 = double(simOut.ref_signal.Time(:));
    x1 = double(simOut.ref_signal.Data(:));       
    t2 = double(simOut.out_signal.Time(:));
    y2 = double(simOut.out_signal.Data(:));      
    
    t_start = max(t1(1), t2(1));
    t_end   = min(t1(end), t2(end));
    idx1 = t1 >= t_start & t1 <= t_end;
    t = t1(idx1);
    x = x1(idx1);
    
    y = interp1(t2, y2, t, 'linear');
    
    valid = isfinite(t) & isfinite(x) & isfinite(y);
    t = t(valid);
    x = x(valid);
    y = y(valid);
    
    t_skip = 0;  
    idx = t >= (t(1) + t_skip);
    t = t(idx);
    x = x(idx);
    y = y(idx);
    
    if numel(t) < 3
        error('Недостаточно данных для корректного расчета.');
    end
    
    e = x - y;
    
    MAE      = mean(abs(e));
    MSE      = mean(e.^2);
    RMSE     = sqrt(MSE);
    MaxError = max(abs(e));
    Bias     = mean(e);
    StdError = std(e);
    
    if std(x) > eps && std(y) > eps
        C = corrcoef(x, y);
        r_cor = C(1,2);
    else
        r_cor = NaN;
    end
    
    t_rel = t - t(1);
    IAE  = trapz(t, abs(e));
    ISE  = trapz(t, e.^2);
    ITAE = trapz(t, t_rel .* abs(e));
    ITSE = trapz(t, t_rel .* e.^2);
    
    % Полная вариация ошибки (степень колебательности)
    TV_error = sum(abs(diff(e)));
    
    signal_energy = trapz(t, x.^2);
    error_energy  = trapz(t, e.^2);
    if error_energy > eps && signal_energy > eps
        TrackingSNR_dB = 10*log10(signal_energy/error_energy);
    else
        TrackingSNR_dB = NaN;
    end
    
    % Оценка высокочастотных шумов в ошибке управления
    dt = median(diff(t));
    fs = 1/dt;
    hf_cutoff = 20; 
    if hf_cutoff < fs/2
        e_low = lowpass(e, hf_cutoff, fs);
        e_high = e - e_low;
        HF_RMS = sqrt(mean(e_high.^2));
        HF_Peak = max(abs(e_high));
    else
        HF_RMS = NaN;
        HF_Peak = NaN;
    end
    
    U_RMS = NaN; U_Max = NaN; U_Energy = NaN; TV_control = NaN;
    
    metricNames = {'r_cor'; 'MAE'; 'RMSE'; 'MaxError'; 'Bias'; 'StdError'; ...
                   'IAE'; 'ISE'; 'ITAE'; 'ITSE'; 'TV_error'; 'TrackingSNR_dB'; ...
                   'HF_RMS'; 'HF_Peak'; 'U_RMS'; 'U_Max'; 'U_Energy'; 'TV_control'};
               
    metricValues = [r_cor; MAE; RMSE; MaxError; Bias; StdError; ...
                    IAE; ISE; ITAE; ITSE; TV_error; TrackingSNR_dB; ...
                    HF_RMS; HF_Peak; U_RMS; U_Max; U_Energy; TV_control];
                
    metrics = table(metricNames, metricValues, 'VariableNames', {'Metric', 'Value'});
end