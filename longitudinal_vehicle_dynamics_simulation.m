function longitudinal_vehicle_dynamics_simulation()
    % Main simulation function for longitudinal vehicle dynamics
    
    % Vehicle parameters
    params.m = 1500;       % Vehicle mass [kg]
    params.g = 9.81;       % Gravitational acceleration [m/s^2]
    params.rho = 1.225;    % Air density [kg/m^3]
    params.Cd = 0.3;       % Drag coefficient
    params.Af = 2.5;       % Frontal area [m^2]
    params.reff = 0.3;     % Effective wheel radius [m]
    params.f = 0.015;      % Rolling resistance coefficient
    params.h = 0.5;        % CG height [m]
    params.haero = 0.6;    % Aerodynamic center height [m]
    params.lf = 1.2;       % Distance from CG to front axle [m]
    params.lr = 1.5;       % Distance from CG to rear axle [m]
    params.Cqf = 100000;   % Front tire longitudinal stiffness [N]
    params.Cqr = 100000;   % Rear tire longitudinal stiffness [N]
    
    % Initial conditions
    x0 = 0;                % Initial position [m]
    v0 = 20;               % Initial velocity [m/s]
    omega_wf0 = v0/params.reff; % Front wheel angular velocity [rad/s]
    omega_wr0 = v0/params.reff; % Rear wheel angular velocity [rad/s]
    
    initial_state = [x0; v0; omega_wf0; omega_wr0];
    
    % Simulation time
    tspan = [0 10];        % Simulation time [s]
    
    % Road slope (0 for flat road)
    theta = 0;             % Road slope angle [rad]
    
    % Wind speed
    Vwind = 0;             % Wind speed [m/s] (positive for headwind)
    
    % Torque inputs (simplified - could be made more sophisticated)
    Tf = 0;                % Front wheel torque [Nm] (positive for acceleration)
    Tr = 0;                % Rear wheel torque [Nm]
    
    % Combine additional parameters
    params.theta = theta;
    params.Vwind = Vwind;
    params.Tf = Tf;
    params.Tr = Tr;
    
    % Solve the ODE
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
    [t, state] = ode45(@(t, y) vehicle_dynamics(t, y, params), tspan, initial_state, options);
    
    % Extract results
    x = state(:, 1);
    v = state(:, 2);
    omega_wf = state(:, 3);
    omega_wr = state(:, 4);
    
    % Calculate additional quantities for analysis
    Fxr = zeros(size(t));
    Fxf = zeros(size(t));
    for i = 1:length(t)
        [~, forces] = vehicle_dynamics(t(i), state(i, :)', params);
        Fxf(i) = forces.Fxf;
        Fxr(i) = forces.Fxr;
    end
    
    % Plot results
    figure;
    subplot(3, 1, 1);
    plot(t, v * 3.6); % Convert to km/h
    xlabel('Time [s]');
    ylabel('Velocity [km/h]');
    title('Vehicle Speed');
    grid on;
    
    subplot(3, 1, 2);
    plot(t, Fxf, 'b', t, Fxr, 'r');
    xlabel('Time [s]');
    ylabel('Tire Forces [N]');
    legend('Front tire force', 'Rear tire force');
    title('Longitudinal Tire Forces');
    grid on;
    
    subplot(3, 1, 3);
    plot(t, x);
    xlabel('Time [s]');
    ylabel('Position [m]');
    title('Vehicle Position');
    grid on;
end

function [dstate, forces] = vehicle_dynamics(t, state, params)
    % Vehicle dynamics equations
    
    % Unpack state variables
    x = state(1);       % Position (not used directly in dynamics)
    v = state(2);       % Velocity
    omega_wf = state(3); % Front wheel angular velocity
    omega_wr = state(4); % Rear wheel angular velocity
    
    % Unpack parameters
    m = params.m;
    g = params.g;
    rho = params.rho;
    Cd = params.Cd;
    Af = params.Af;
    reff = params.reff;
    f = params.f;
    h = params.h;
    haero = params.haero;
    lf = params.lf;
    lr = params.lr;
    Cqf = params.Cqf;
    Cqr = params.Cqr;
    theta = params.theta;
    Vwind = params.Vwind;
    Tf = params.Tf;
    Tr = params.Tr;
    
    % Calculate aerodynamic drag force
    Faero = 0.5 * rho * Cd * Af * (v + Vwind)^2;
    
    % Calculate normal forces (simplified static distribution)
    Fzf = (-Faero * haero - m * 0 * h - m * g * h * sin(theta) + m * g * lr * cos(theta)) / (lf + lr);
    Fzr = (Faero * haero + m * 0 * h + m * g * h * sin(theta) + m * g * lf * cos(theta)) / (lf + lr);
    
    % Determine if accelerating or braking based on wheel speeds
    is_accelerating = (Tf + Tr) > 0;
    
    % Calculate longitudinal slip ratios
    if is_accelerating
        sigma_xf = (reff * omega_wf - v) / (reff * omega_wf);
        sigma_xr = (reff * omega_wr - v) / (reff * omega_wr);
    else
        sigma_xf = (reff * omega_wf - v) / v;
        sigma_xr = (reff * omega_wr - v) / v;
    end
    
    % Calculate longitudinal tire forces
    Fxf = Cqf * sigma_xf;
    Fxr = Cqr * sigma_xr;
    
    % Calculate rolling resistance
    Rxf = f * Fzf;
    Rxr = f * Fzr;
    
    % Total longitudinal force
    F_total = Fxf + Fxr - Faero - Rxf - Rxr - m * g * sin(theta);
    
    % Vehicle acceleration
    v_dot = F_total / m;
    
    % Wheel dynamics (simplified)
    Iw = 2.0; % Wheel inertia [kg*m^2] - should be adjusted for real vehicle
    
    % Front wheel angular acceleration
    omega_wf_dot = (Tf - Fxf * reff) / Iw;
    
    % Rear wheel angular acceleration
    omega_wr_dot = (Tr - Fxr * reff) / Iw;
    
    % State derivatives
    dstate = [v;         % dx/dt = velocity
              v_dot;     % dv/dt = acceleration
              omega_wf_dot; % d(omega_wf)/dt
              omega_wr_dot]; % d(omega_wr)/dt
    
    % Optional: return force components for analysis
    if nargout > 1
        forces.Fxf = Fxf;
        forces.Fxr = Fxr;
        forces.Faero = Faero;
        forces.Rxf = Rxf;
        forces.Rxr = Rxr;
        forces.F_total = F_total;
    end
end