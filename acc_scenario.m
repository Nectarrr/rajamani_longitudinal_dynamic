function acc_scenario_with_engagement()
    % Adaptive Cruise Control: демонстрация включения в t_on
    % Параметры автомобиля
    p.m    = 1500;    % кг
    p.g    = 9.81;    % м/с^2
    p.rho  = 1.225;   % кг/м^3
    p.Cd   = 0.3;
    p.Af   = 2.5;     % м^2
    p.reff = 0.3;     % м
    p.f    = 0.015;   % коэф. качения
    p.Iw   = 2.0;     % кг·м^2

    % Параметры ACC
    d0      = 5;      % м, минимальная дистанция
    h_head  = 1.5;    % с, запас по времени
    Kp      = 0.5;    % П‑коэф.
    Ki      = 0.1;    % И‑коэф.
    a_max   =  2;     % м/с^2
    a_min   = -4;     % м/с^2

    % Время симуляции
    t_end = 80; dt = 0.1;
    t = (0:dt:t_end)';
    N = numel(t);

    % Время включения ACC
    t_on = 4;  % с

    % Профиль лидера: синусоида вокруг 30 м/с
    v_set = 30; A = 2.5; T = 40;
    omega = 2*pi/T;
    v_lead = v_set + A*sin(omega*t);
    a_lead =        A*omega*cos(omega*t);
    x_lead = cumtrapz(t, v_lead);

    % Инициализация эго-авто
    x = zeros(N,1);
    v = zeros(N,1);
    % задаём начальную скорость отличную от лидера
    v(1) = v_set - 5; 
    % начальное расстояние больше безопасного на +10 м
    x(1) = x_lead(1) - (d0 + h_head*v(1)) + 10;

    % Хранилища
    a_ego      = zeros(N,1);
    Tf_arr     = zeros(N,1);
    integral_e = 0;

    % Основной цикл интегрирования (прямой Эйлер)
    for i = 1:N-1
        % текущее расстояние и желаемая дистанция
        d_rel = x_lead(i) - x(i);
        d_des = d0 + h_head * v(i);
        e = d_rel - d_des;

        if t(i) < t_on
            % ACC выключен
            Tf = 0;
        else
            % ACC включён: PI‑контроллер с feed‑forward
            integral_e = integral_e + e*dt;
            a_des = a_lead(i) + Kp*e + Ki*integral_e;
            a_des = min(max(a_des, a_min), a_max);
            Tf = p.m * a_des * p.reff;
        end

        Tf_arr(i) = Tf;

        % сопротивления
        Faero  = 0.5 * p.rho * p.Cd * p.Af * v(i)^2;
        Froll = p.f * p.m * p.g;

        % итоговая сила и ускорение
        Fdrive = Tf / p.reff;
        v_dot  = (Fdrive - Faero - Froll) / p.m;
        a_ego(i) = v_dot;

        % шаг интегрирования
        x(i+1) = x(i) + v(i)*dt;
        v(i+1) = v(i) + v_dot*dt;
    end
    % выравниваем последний точечный
    a_ego(end)  = a_ego(end-1);
    Tf_arr(end) = Tf_arr(end-1);

    % Построение графиков
    figure('Position',[100 100 600 800]);

    % 1) Ускорение
    subplot(3,1,1);
    plot(t, a_ego,  'r','LineWidth',1.5); hold on
    plot(t, a_lead,'b--','LineWidth',1.5);
    xline(t_on,'k:','ACC on','LabelOrientation','horizontal','LabelVerticalAlignment','bottom');
    hold off
    xlabel('Время (с)'), ylabel('м/с^2');
    title('Ускорение');
    legend('ТС с ACC','Целевое ТС','Location','best');
    grid on;

    % 2) Скорости
    subplot(3,1,2);
    plot(t, v,      'r','LineWidth',1.5); hold on
    plot(t, v_lead, 'b--','LineWidth',1.5);
    plot([t(1),t(end)], [v_set,v_set],'k:','LineWidth',1.2);
    xline(t_on,'k:');
    hold off
    xlabel('Время (с)'), ylabel('м/с');
    title('Скорость');
    legend('ТС с ACC','Целевое ТС','set','Location','best');
    grid on;

    % 3) Дистанция
    subplot(3,1,3);
    plot(t, x_lead - x,          'r','LineWidth',1.5); hold on
    plot(t, d0 + h_head.*v,     'b--','LineWidth',1.5);
    xline(t_on,'k:');
    hold off
    xlabel('Время (с))'), ylabel('м');
    title('Дистанция между двумя автомобилями');
    legend('Текущая','Безопасная','Location','best');
    grid on;
end
