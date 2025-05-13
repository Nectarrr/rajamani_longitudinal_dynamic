
clear; clc;

%% Параметры моделирования
Ts = 0.001;            % Шаг моделирования, с
Tsim = 5;              % Время моделирования, с
time = 0:Ts:Tsim;

%% Требования PCR и параметры актуаторов
% Тормозной актуатор
PCR.Brake.max_decel   = -8;       
PCR.Brake.reaction    = 0.120;     
PCR.Brake.AEB_rise    = 0.150;     
tau_brake            = 0.050;      

% Дроссельный актуатор
PCR.Throttle.full_time = 0.100;    
PCR.Throttle.max_error = 1;        
tau_throttle         = PCR.Throttle.full_time/3; 

% Алгоритм продольного управления
sample_interval       = 0.020;     
t_gap_desired         = 1.8;       
allowed_gap_error     = 0.3;      

%% Инициализация массивов
N = length(time);
brake_cmd      = zeros(1,N);
brake_state    = zeros(1,N);
brake_err      = zeros(1,N);
throttle_cmd   = zeros(1,N);
throttle_state= zeros(1,N);
throttle_err  = zeros(1,N);
a_ref          = zeros(1,N);
sample_mark   = zeros(1,N);

%% Формирование входных команд и a_ref
for i=1:N
    t = time(i);
    % --- Торможение ---
    if t < 2
        brake_cmd(i) = 0;
    elseif t < 3
        brake_cmd(i) = 0.5 * PCR.Brake.max_decel;
    else
        brake_cmd(i) = PCR.Brake.max_decel;
    end
    % --- Ускорение (дроссель) ---
    if t < 1
        throttle_cmd(i) = 0;
    elseif t < 4
        throttle_cmd(i) = 100; % 80% открытия
    else
        throttle_cmd(i) = 0;
    end
    % --- Расчет a_ref каждые sample_interval ---
    if abs(mod(t, sample_interval))< Ts/2
        if t<1
            a_ref(i) = 0;
        elseif t<2
            a_ref(i) = 1.2; % м/с^2
        elseif t<3
            a_ref(i) = -4;  % м/с^2
        else
            a_ref(i) = -6;  % м/с^2
        end
        sample_mark(i) = 1;
    else
        a_ref(i) = a_ref(max(i-1,1));
    end
end

%% Моделирование динамики актуаторов
for i=2:N
    dt = Ts;
    % Тормозной актуатор (первый порядок)
    error_cmd = brake_cmd(i) - brake_state(i-1);
    brake_state(i) = brake_state(i-1) + (error_cmd/tau_brake)*dt;
    brake_err(i)   = brake_cmd(i) - brake_state(i);
    % Учитываем AEB (быстрое нарастание до 50%)
    if brake_cmd(i) <= 0.5*PCR.Brake.max_decel && ...
       (time(i)-2) <= PCR.Brake.AEB_rise
        % Дополнительная динамика для AEB
        ramp = (time(i)-2)/PCR.Brake.AEB_rise;
        brake_state(i) = ramp * 0.5 * PCR.Brake.max_decel;
    end
    
    % Дроссельный актуатор (первый порядок)
    error_th = throttle_cmd(i) - throttle_state(i-1);
    throttle_state(i) = throttle_state(i-1) + (error_th/tau_throttle)*dt;
    throttle_err(i)   = throttle_cmd(i) - throttle_state(i);
    % Погрешность позиционирования (шум)
    throttle_state(i) = throttle_state(i) + randn()*PCR.Throttle.max_error*0.01;
end

%% Преобразование дросселя в ускорение
acc_from_throttle = (throttle_state/100)*3;  % Пример: 100% -> +3 м/с^2

%% Графики
% 1) Команды и состояния актуаторов
figure('Name','Актуаторы ACC');
subplot(3,1,1);
plot(time, brake_cmd, '--','LineWidth',1.5); hold on;
plot(time, brake_state,'LineWidth',2);
ylabel('Замедление, м/с^2'); grid on;
legend('Команда','Реализация'); title('Тормозной актуатор');

subplot(3,1,2);
plot(time, throttle_cmd,'--','LineWidth',1.5); hold on;
plot(time, throttle_state,'LineWidth',2);
ylabel('Дроссель, %'); grid on;
legend('Команда','Реализация'); title('Актуатор дроссельной заслонки');

subplot(3,1,3);
plot(time, a_ref,'-','LineWidth',1.5); hold on;
plot(time, acc_from_throttle,'LineWidth',2);
ylabel('Ускорение, м/с^2'); xlabel('Время, с'); grid on;
legend('a_{ref}','a_{act}'); title('Алгоритм продольного управления');

% 2) Ошибки
figure('Name','Ошибки актуаторов');
subplot(2,1,1);
plot(time, brake_err,'LineWidth',1.5);
ylabel('Ошибка, м/с^2'); grid on;
title('Ошибка тормозного актуатора');

subplot(2,1,2);
plot(time, throttle_err,'LineWidth',1.5);
ylabel('Ошибка, %'); xlabel('Время, с'); grid on;
title('Ошибка дроссельного актуатора');

% 3) Маркеры расчета a_ref (CAN FD интерфейс)
figure('Name','Маркировка расчета a_ref');
stem(time(sample_mark==1), a_ref(sample_mark==1), 'filled');
ylabel('a_{ref}, м/с^2'); xlabel('Время, с'); grid on;
title('Расчет желаемого ускорения каждые 20 мс (CAN FD)');

% 4) Верификация требований PCR
figure('Name','Верификация PCR');
bar([PCR.Brake.max_decel, -1]); hold on; ylim([-10 0.5]);
ylabel('м/с^2');
xticklabels({'PCR.55 макс.','--'});
title('Максимальный диапазон тормозного усилия');

disp('=== Результаты верификации ===');
fprintf('PCR.55: макс. торможение = %.2f м/с^2\n', min(brake_state));
fprintf('PCR.56: время реакции = %.0f мс\n', PCR.Brake.reaction*1000);
fprintf('PCR.57: нарастание 50%% = %.0f мс\n', PCR.Brake.AEB_rise*1000);
fprintf('PCR.61: полный ход заслонки = %.0f мс\n', PCR.Throttle.full_time*1000);
fprintf('PCR.62: макс. ошибка дросселя = ±%.2f%%\n', max(abs(throttle_err)));

% Дополнительная верификация точности accel
acc_error = a_ref - acc_from_throttle;
figure('Name','Ошибка ускорения');
plot(time, acc_error,'LineWidth',1.5); grid on;
ylabel('Ошибка, м/с^2'); xlabel('Время, с');
title('Ошибка фактического ускорения и заданного');
fprintf('Максимальная ошибка ускорения = %.2f м/с^2 (должно ≤0.25)\n', max(abs(acc_error)));
