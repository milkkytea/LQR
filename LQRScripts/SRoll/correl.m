% --- Данные из Simulink ---
t1 = com.Time;
x  = com.Data(:);   % эталонный сигнал

t2 = real.Time;
y  = real.Data(:);   % сравниваемый сигнал

% --- Оставляем общий участок времени ---
t_start = max(t1(1), t2(1));
t_end   = min(t1(end), t2(end));

idx = t1 >= t_start & t1 <= t_end;

t = t1(idx);
x = x(idx);

% --- Интерполируем второй сигнал на сетку первого ---
y = interp1(t2, y, t, 'linear');

% --- Убираем NaN, если они появились ---
valid = ~isnan(x) & ~isnan(y);

t = t(valid);
x = x(valid);
y = y(valid);

% --- Если нужно отбросить начальный переходный процесс ---
t_skip = 0; % например, 2 если нужно отбросить первые 2 секунды

idx = t >= t_skip;

t = t(idx);
x = x(idx);
y = y(idx);

% --- Ошибка ---
e = x - y;

% --- Корреляция ---
R = corrcoef(x, y);
rh = R(1,2);

% --- Метрики ошибки ---
MAE  = mean(abs(e));
MSE  = mean(e.^2);
RMSE = sqrt(mean(e.^2));
MaxError = max(abs(e));

% --- SNR, дБ ---
signal_power = sum(x.^2);
noise_power  = sum(e.^2);

SNR_dB = 10*log10(signal_power / noise_power);

% --- Вывод результатов ---
fprintf('Correlation rho = %.6f\n', rh);
fprintf('MAE             = %.6f\n', MAE);
fprintf('MSE             = %.6f\n', MSE);
fprintf('RMSE            = %.6f\n', RMSE);
fprintf('Max Error       = %.6f\n', MaxError);
fprintf('SNR             = %.6f dB\n', SNR_dB);