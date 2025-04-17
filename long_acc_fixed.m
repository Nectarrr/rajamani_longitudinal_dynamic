function acc_plots_synchronous()
    %===== Параметры автомобиля =====
    p.m    = 1500;     % масса, кг
    p.g    = 9.81;     % гравитация, м/с^2
    p.rho  = 1.225;    % плотность воздуха, кг/м^3
    p.Cd   = 0.3;      % Cd
    p.Af   = 2.5;      % м^2
    p.reff = 0.3;      % м
    p.f    = 0.015;    % rolling resistance coeff
    p.Iw   = 2.0;      % кг·м^2

    %===== Параметры ACC =====
    d0      = 5;       % минимальная дистанция, м
    h_head  = 1.5;     % запас по времени, с
    Kp      = 0.5;     % проп.
    Ki      = 0.1;     % интегр.
    a_max   =  2;      % макс. ускор.
    a_min   = -4;      % макс. торм.

    %===== Профиль лидера =====
    t_end = 80; dt = 0.1;
    t = (0:dt:t_end)';
    N = numel(t);
    v_set = 30;            % м/с
    A     = 2.5;           % ампл. синусоиды, м/с
    T     = 40;            % период, с

    % скорость и ускорение лидера (аналитически)
    omega = 2*pi/T;
    v_lead = v_set + A*sin(omega*t);
    a_lead =      A*omega*cos(omega*t);
    % позиция лидера
    x_lead = cumtrapz(t, v_lead);

    %===== Инициализация эго-авто =====
    x = zeros(N,1);
    v = zeros(N,1);
    v(1) = v_lead(1);        % стартуем на той же скорости
    % чтобы сразу держать safe‐distance:
    x(1) = x_lead(1) - (d0 + h_head*v(1));

    % заготовки под логику и графики
    a_ego   = zeros(N,1);
    Tf_arr  = zeros(N,1);
    integral_e = 0;

    %===== Цикл интегрирования =====
    for i = 1:N-1
        % текущее расстояние и желаемое
        d_rel = x_lead(i) - x(i);
        d_des = d0 + h_head * v(i);
        e = d_rel - d_des;
        integral_e = integral_e + e*dt;

        % feed‑forward + PI
        a_des = a_lead(i) + Kp*e + Ki*integral_e;
        a_des = min(max(a_des, a_min), a_max);

        % переводим в крутящий момент
        Tf = p.m * a_des * p.reff;
        Tf_arr(i) = Tf;

        % сопротивления
        Faero  = 0.5 * p.rho * p.Cd * p.Af * v(i)^2;
        Froll = p.f * p.m * p.g;

        % итоговая сила и ускорение
        Fdrive = Tf / p.reff;
        v_dot = (Fdrive - Faero - Froll) / p.m;
        a_ego(i) = v_dot;

        % Эйлер-прямой
        x(i+1) = x(i) + v(i)*dt;
        v(i+1) = v(i) + v_dot*dt;
    end
    % последний элемент
    a_ego(end)  = a_ego(end-1);
    Tf_arr(end) = Tf_arr(end-1);

    %===== Рисуем три субплота =====
    figure('Position',[100 100 600 800]);

    % 1) Acceleration
    subplot(3,1,1)
    plot(t, a_ego,  'r','LineWidth',1.5); hold on
    plot(t, a_lead,'b--','LineWidth',1.5); hold off
    xlabel('time (sec)'), ylabel('m/s^2')
    title('Acceleration')
    legend('ego','lead','Location','best')
    grid on

    % 2) Velocity
    subplot(3,1,2)
    plot(t, v,      'r','LineWidth',1.5); hold on
    plot(t, v_lead, 'b--','LineWidth',1.5);
    plot(t, v_set*ones(N,1),'k:','LineWidth',1.2); hold off
    xlabel('time (sec)'), ylabel('m/s')
    title('Velocity')
    legend('ego','lead','set','Location','best')
    grid on

    % 3) Distance
    subplot(3,1,3)
    plot(t, x_lead - x,           'r','LineWidth',1.5); hold on
    plot(t, d0 + h_head.*v,      'b--','LineWidth',1.5); hold off
    xlabel('time (sec)'), ylabel('m')
    title('Distance between two cars')
    legend('actual','safe','Location','best')
    grid on
end
