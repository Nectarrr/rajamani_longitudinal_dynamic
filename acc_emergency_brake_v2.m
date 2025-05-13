function acc_with_emergency_and_initial_boost()
    %=== Параметры автомобиля ===
    p.m    = 1500;    % кг
    p.g    = 9.81;    % м/с^2
    p.rho  = 1.225;   % кг/м^3
    p.Cd   = 0.3;
    p.Af   = 2.5;     % м^2
    p.reff = 0.3;     % м
    p.f    = 0.015;   % коэф. качения
    p.Iw   = 2.0;     % кг·м^2

    %=== Параметры ACC ===
    d0       = 5;       % м
    h_head   = 1.5;     % с
    safe_dist= d0 + h_head*30;  % 5 + 1.5*30 = 50м, постоянная safe
    Kp       = 0.5;
    Ki       = 0.1;
    a_max    =  2;      % м/с^2
    a_min    = -4;      % обычное торм.
    a_min_em = -8;      % экстренное торм.

    %=== Время ===
    t_end   = 80; dt=0.1;
    t       = (0:dt:t_end)';
    N       = numel(t);
    t_on    = 10;    % с — включение ACC
    t_brake = 60;    % с — начало экстренного торм.

    %=== Профиль лидера (синусоида + экстр. торм.) ===
    v_set = 30; A=2.5; T=40; w=2*pi/T;
    v_lead = zeros(N,1);
    a_lead = zeros(N,1);
    x_lead = zeros(N,1);
    v_lead(1)=v_set + A*sin(0);

    %=== Инициализация эго-авто ===
    v = zeros(N,1);
    x = zeros(N,1);
    v(1) = 20;        
    x(1) = x_lead(1) - safe_dist + 0;  % дистанция ≈ safe на старте

    %=== Заготовки ===
    a_ego  = zeros(N,1);
    Tf_arr = zeros(N,1);
    integral_e = 0;

    %=== Цикл интегрирования ===
    for i=1:N-1
        %-- лидер --
        if t(i)<t_brake
            a_lead(i) = A*w*cos(w*t(i));
        elseif t(i)<t_brake+2
            a_lead(i) = -8;
        else
            a_lead(i) = 0;
        end
        v_lead(i+1) = max(v_lead(i) + a_lead(i)*dt,0);
        x_lead(i+1) = x_lead(i) + v_lead(i)*dt;

        %-- контроль --
        if t(i)<t_on
            % до 10с — постоянный «буст» a=2
            a_des = 2;
        else
            % ACC: feed-forward + PI
            d_rel = x_lead(i)-x(i);
            e     = d_rel - (d0 + h_head*v(i));
            integral_e = integral_e + e*dt;
            a_des = a_lead(i) + Kp*e + Ki*integral_e;
        end
        % ограничение по торможению
        if t(i)>=t_brake
            amin_curr = a_min_em;
        else
            amin_curr = a_min;
        end
        a_des = min(max(a_des,amin_curr),a_max);

        % крутящий момент
        Tf = p.m * a_des * p.reff;
        Tf_arr(i)=Tf;

        %-- динамика эго --
        Faero  = 0.5*p.rho*p.Cd*p.Af*v(i)^2;
        Froll = p.f * p.m * p.g;
        Fdrive= Tf / p.reff;
        v_dot  = (Fdrive - Faero - Froll)/p.m;
        a_ego(i)=v_dot;

        % интегрируем
        v(i+1)= max(v(i)+v_dot*dt,0);
        x(i+1)= x(i)+v(i)*dt;
    end
    a_lead(end)=a_lead(end-1);
    a_ego(end) =a_ego(end-1);

    %=== Рисуем графики ===
    figure('Position',[100 100 600 800]);

    % 1) Acceleration
    subplot(3,1,1);
    plot(t,a_ego,'r','LineWidth',1.5); hold on
    plot(t,a_lead,'b--','LineWidth',1.5);
    xline(t_on,   'k:','ACC on','LabelVerticalAlignment','bottom');
    xline(t_brake,'k:','Emergency','LabelVerticalAlignment','bottom');
    hold off
    xlabel('time (sec)'), ylabel('m/s^2');
    title('Acceleration'); legend('ego','lead','Location','best');
    grid on

    % 2) Velocity
    subplot(3,1,2);
    plot(t,v,'r','LineWidth',1.5); hold on
    plot(t,v_lead,'b--','LineWidth',1.5);
    plot([t(1),t(end)],[v_set,v_set],'k:','LineWidth',1.2);
    xline(t_on,'k:'), xline(t_brake,'k:');
    hold off
    xlabel('time (sec)'), ylabel('m/s');
    title('Velocity'); legend('ego','lead','set','Location','best');
    grid on

    % 3) Distance
    subplot(3,1,3);
    plot(t,x_lead-x,'r','LineWidth',1.5); hold on
    plot(t,safe_dist*ones(N,1),'b--','LineWidth',1.5);
    xline(t_on,'k:'), xline(t_brake,'k:');
    hold off
    xlabel('time (sec)'), ylabel('m');
    title('Distance between two cars');
    legend('actual','safe','Location','best');
    grid on
end
