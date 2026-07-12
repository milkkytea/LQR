%% Исходные данные
t1 = double(com.Time(:));
x1 = double(com.Data(:));       % заданный сигнал

t2 = double(real.Time(:));
y2 = double(real.Data(:));      % фактический сигнал

%% Общий временной диапазон
t_start = max(t1(1), t2(1));
t_end   = min(t1(end), t2(end));

idx1 = t1 >= t_start & t1 <= t_end;

t = t1(idx1);
x = x1(idx1);

%% Интерполяция фактического сигнала
y = interp1(t2, y2, t, 'linear');

%% Удаление некорректных значений
valid = isfinite(t) & isfinite(x) & isfinite(y);

t = t(valid);
x = x(valid);
y = y(valid);

%% Отбрасывание начала записи
t_skip = 0;  % секунды от начала общего участка

idx = t >= (t(1) + t_skip);

t = t(idx);
x = x(idx);
y = y(idx);

if numel(t) < 3
    error('Недостаточно данных после обработки.');
end

%% Ошибка
e = x - y;

%% Базовые метрики
MAE      = mean(abs(e));
MSE      = mean(e.^2);
RMSE     = sqrt(MSE);
MaxError = max(abs(e));
Bias     = mean(e);
StdError = std(e);

%% Корреляция
if std(x) > eps && std(y) > eps
    C = corrcoef(x, y);
    r_cor = C(1,2);
else
    r_cor = NaN;
end

%% Интегральные критерии
t_rel = t - t(1);

IAE  = trapz(t, abs(e));
ISE  = trapz(t, e.^2);
ITAE = trapz(t, t_rel .* abs(e));
ITSE = trapz(t, t_rel .* e.^2);

%% Полная вариация ошибки
TV_error = sum(abs(diff(e)));

%% Отношение энергии задания к энергии ошибки
signal_energy = trapz(t, x.^2);
error_energy  = trapz(t, e.^2);

if error_energy > eps && signal_energy > eps
    TrackingSNR_dB = 10*log10(signal_energy/error_energy);
else
    TrackingSNR_dB = NaN;
end

%% Оценка высокочастотной активности ошибки
dt = median(diff(t));
fs = 1/dt;

hf_cutoff = 20; % Гц; подобрать под динамику контура

if hf_cutoff < fs/2
    e_low = lowpass(e, hf_cutoff, fs);
    e_high = e - e_low;

    HF_RMS = sqrt(mean(e_high.^2));
    HF_Peak = max(abs(e_high));
else
    HF_RMS = NaN;
    HF_Peak = NaN;
end

%% Метрики управляющего сигнала, если он существует
have_control = exist('control', 'var') && ...
               ~isempty(control.Time) && ...
               ~isempty(control.Data);

if have_control
    tu = double(control.Time(:));
    u0 = double(control.Data(:));

    u = interp1(tu, u0, t, 'linear');

    valid_u = isfinite(u);
    u_valid = u(valid_u);
    t_valid = t(valid_u);

    U_RMS  = sqrt(mean(u_valid.^2));
    U_Max  = max(abs(u_valid));
    U_Energy = trapz(t_valid, u_valid.^2);
    TV_control = sum(abs(diff(u_valid)));
else
    U_RMS = NaN;
    U_Max = NaN;
    U_Energy = NaN;
    TV_control = NaN;
end

%% Таблица результатов
metricNames = {
    'r_cor'
    'MAE'
    'RMSE'
    'MaxError'
    'Bias'
    'StdError'
    'IAE'
    'ISE'
    'ITAE'
    'ITSE'
    'TV_error'
    'TrackingSNR_dB'
    'HF_RMS'
    'HF_Peak'
    'U_RMS'
    'U_Max'
    'U_Energy'
    'TV_control'
};

metricValues = [
    r_cor
    MAE
    RMSE
    MaxError
    Bias
    StdError
    IAE
    ISE
    ITAE
    ITSE
    TV_error
    TrackingSNR_dB
    HF_RMS
    HF_Peak
    U_RMS
    U_Max
    U_Energy
    TV_control
];

metrics = table(metricNames, metricValues, ...
    'VariableNames', {'Metric', 'Value'});

disp(metrics);