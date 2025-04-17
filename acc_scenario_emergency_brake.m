function acc_scenario_with_emergency_brake()
    % Сценарий ACC: выключен → включен → экстренное торможение лидера

    %====== Параметры автомобиля ======
    p.m    = 1500;    % кг
    p.g    = 9.81;    % м/с^2
    p.rho  = 1.225;   % кг/м^3
    p.Cd   = 0.3;
    p.Af   = 2.5;     % м^2
    p.reff = 0.3;     % м
    p.f    = 0.015;   % коэф. качения
    p.Iw   = 2.0;     % кг·м^2

    %====== Параметры ACC ======
    d0      = 5;        % м, минимальная дистанция
    h_head  = 1.5;      % с, запас по времени
    Kp      = 0.5;      % П‑коэф.
    Ki      = 0.1;      % И‑коэф.
    a_max   =  2;       % макс. ускорение, м/с^2
    a_min   = -4;       % макс. обычное торможение, м/с^2
    a_min_em = -8;      % макс. экстренное торможение, м/с^2

    %====== Время и события ======
    t_end   = 80; dt = 0.1;
    t       = (0:dt:t_end)';
    N       = numel(t);
    t_on    = 4;   % время включения ACC, с
    t_brake = 60;  % момент экстренного торможения лидера, с

    %====== Профиль лидера ======
    v_set = 30;           % м/с
    A     = 2.5;          % ампл. синусоиды, м/с
    T     = 40;           % период, с
    omega = 2*pi/T;

    x_lead = zeros(N,1);
    v_lead = zeros(N,1);
    a_lead = zeros(N,1);
    % начальные условия лидера
    v_lead(1) = v_set + A*sin(omega*0);
    x_lead(1) = 0;

    %====== Инициализация эго-авто ======
    x = zeros(N,1);
    v = zeros(N,1);
    v(1) = v_set - 5;  % стартуем медленнее лидера
    x(1) = x_lead(1) - (d0 + h_head*v(1)) + 10;  % на 10 м дальше

    %====== Заготовки для результатов ======
    a_ego      = zeros(N,1);
    Tf_arr     = zeros(N,1);
    integral_e = 0;

    %====== Основной цикл ======
    for i = 1:N-1
        %--- Обновление лидера ---
        if t(i) < t_brake
            % обычный синусоидальный режим
            a_lead(i) = A*omega*cos(omega*t(i));
        elseif t(i) < t_brake + 2
            % экстренное торможение в течение 2 с
            a_lead(i) = -8;
        else
            % дальше стоит на месте (или держит скорость)
            a_lead(i) = 0;
        end
        v_lead(i+1) = max(v_lead(i) + a_lead(i)*dt, 0);
        x_lead(i+1) = x_lead(i) + v_lead(i)*dt;

        %--- Расчёт ошибки по дистанции ---
        d_rel = x_lead(i) - x(i);
        d_des = d0 + h_head * v(i);
        e     = d_rel - d_des;

        %--- Логика ACC ---
        if t(i) < t_on
            % ACC выключен
            Tf = 0;
        else
            % ACC включен
            integral_e = integral_e + e*dt;
            a_des = a_lead(i) + Kp*e + Ki*integral_e;
            % выбираем ограничение торможения
            amin_curr = (t(i) < t_brake) * a_min + (t(i) >= t_brake) * a_min_em;
            a_des = min(max(a_des, amin_curr), a_max);
            Tf = p.m * a_des * p.reff;
        end
        Tf_arr(i) = Tf;

        %--- Силы сопротивления и ускорение эго-авто ---
        Faero  = 0.5 * p.rho * p.Cd * p.Af * v(i)^2;
        Froll = p.f * p.m * p.g;
        Fdrive = Tf / p.reff;
        v_dot  = (Fdrive - Faero - Froll) / p.m;
        a_ego(i) = v_dot;

        %--- Интегрирование состояния эго-авто ---
        x(i+1) = x(i) + v(i)*dt;
        v(i+1) = max(v(i) + v_dot*dt, 0);
    end
    % Выравниваем последний элемент
    a_lead(end) = a_lead(end-1);
    a_ego(end)  = a_ego(end-1);
    Tf_arr(end) = Tf_arr(end-1);

    %====== Построение графиков ======
    figure('Position',[100 100 600 800]);

    % 1) Ускорение
    subplot(3,1,1);
    plot(t, a_ego,  'r','LineWidth',1.5); hold on
    plot(t, a_lead,'b--','LineWidth',1.5);
    xline(t_on,   'k:','ACC on','LabelVerticalAlignment','bottom');
    xline(t_brake,'k:','Экстренная ситуация','LabelVerticalAlignment','bottom');
    hold off
    xlabel('Время (с)'), ylabel('м/с^2');
    title('Ускорение');
    legend('ТС с ACC','Целевое ТС','Location','best');
    grid on;

    % 2) Скорости
    subplot(3,1,2);
    plot(t, v,      'r','LineWidth',1.5); hold on
    plot(t, v_lead, 'b--','LineWidth',1.5);
    plot([t(1),t(end)],[v_set,v_set],'k:','LineWidth',1.2);
    xline(t_on,   'k:');
    xline(t_brake,'k:');
    hold off
    xlabel('Время (с)'), ylabel('м/с');
    title('Скорость');
    legend('ТС с ACC','Целевое ТС','Установленная','Location','best');
    grid on;

    % 3) Дистанция
    subplot(3,1,3);
    plot(t, x_lead - x,      'r','LineWidth',1.5); hold on
    plot(t, d0 + h_head.*v, 'b--','LineWidth',1.5);
    xline(t_on,   'k:');
    xline(t_brake,'k:');
    hold off
    xlabel('Время (с))'), ylabel('м');
    title('Дистанция между двумя автомобилями');
    legend('Текущая','Безопасная','Location','best');
    grid on;
end
