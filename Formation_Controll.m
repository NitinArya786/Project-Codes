function ode_solver_animation()
    % Define initial conditions for the main system
    z1_0 = 5 - 2*sqrt(2);
    z2_0 = 2*sqrt(2);
    fe_0 = 0;

    % Parameters for the main system
    rho_d = 4;
    f_d = 0.785;
    v_l = 5;
    k_1 = 5;
    k_2 = 5;
    w_l = .1;
    L = rho_d*cos(f_d);
    L_1 = rho_d*sin(f_d);

    % Define initial conditions for the auxiliary system
    xl_0 = 5;
    yl_0 = 0;
    fl_0 = 0;

    % Solve ODEs for the main system
    [t_main, Y] = ode45(@(t, y) ode_equations(t, y, k_1, k_2, rho_d, f_d, v_l, w_l, L, L_1), [0 20], [z1_0; z2_0; fe_0]);

    % Extract z1, z2, and fe from the solution
    z1 = Y(:, 1);
    z2 = Y(:, 2);
    fe = Y(:, 3);

    psi = atan((z2 + L_1 .* cos(fe)) ./ (z1 + L - L_1 .* sin(fe)));
    Rho = (z1 + L) .* cos(psi) + z2 .* sin(psi) - L_1 .* sin(fe - psi);

    % Solve ODEs for the auxiliary system
    [t_aux, H] = ode45(@(t, h) ode_equation(t, h), [0 5], [xl_0; yl_0; fl_0]);

    % Extract xl, yl, and fl from the solution
    xl = H(:, 1);
    yl = H(:, 2);
    fl = H(:, 3);

    % Interpolate Rho and cos(psi) to match the time points of xl
    Rho_interp = interp1(t_main, Rho, t_aux);
    cos_psi_interp = interp1(t_main, cos(psi), t_aux);
    sin_psi_interp = interp1(t_main, sin(psi), t_aux);

    % Perform subtraction only if dimensions are compatible
    if isequal(size(xl), size(Rho_interp)) && isequal(size(Rho_interp), size(cos_psi_interp))
        x_f = xl - Rho_interp.*cos_psi_interp;
        y_f = yl - Rho_interp.*sin_psi_interp;
        x_f_1 = xl - Rho_interp.*cos_psi_interp; 
        y_f_1 = yl + Rho_interp.*sin_psi_interp;
        disp('Subtraction successful.');
    else
        disp('Dimensions are not compatible for subtraction.');
        return;
    end

    % Plot initial state
    figure;
    h = plot(x_f(1), y_f(1), 'b-', xl(1), yl(1), 'r-', x_f_1(1), y_f_1(1), 'g-');
    xlabel('x');
    ylabel('y');
    title('Animation of xf vs yf, xl vs yl, and xf_1 vs yf_1');
    axis([-10 10 -10 10]); % Adjust the axis limits as needed
    grid on;
    legend('xf vs yf', 'xl vs yl', 'xf_1 vs yf_1');
    drawnow;

    % Animation loop
    for i = 2:length(t_aux)
        set(h(1), 'XData', x_f(1:i), 'YData', y_f(1:i)); % Update xf vs yf
        set(h(2), 'XData', xl(1:i), 'YData', yl(1:i)); % Update xl vs yl
        set(h(3), 'XData', x_f_1(1:i), 'YData', y_f_1(1:i)); % Update xf_1 vs yf_1
        title(sprintf('Animation of xf vs yf, xl vs yl, and xf_1 vs yf_1 (t = %.2f)', t_aux(i)));
        drawnow;
        pause(0.1); % Adjust the pause duration as needed
    end

    disp('Animation finished.');

    % Plot main system results
    figure;

    subplot(6,1,1);
    plot(t_main, z1);
    xlabel('Time');
    ylabel('z1');
    title('z1 vs Time');

    subplot(6,1,2);
    plot(t_main, z2);
    xlabel('Time');
    ylabel('z2');
    title('z2 vs Time');

    subplot(6,1,3);
    plot(t_main, fe);
    xlabel('Time');
    ylabel('fe');
    title('fe vs Time');

    subplot(6,1,4);
    plot(t_main, psi);
    xlabel('Time');
    ylabel('psi');
    title('psi vs Time');

    subplot(6,1,5);
    plot(t_main, Rho);
    xlabel('Time');
    ylabel('Rho');
    title('Rho vs Time');
end

function dydt = ode_equation(~, h)
    xl = h(1);
    yl = h(2); 
    fl = h(3);

    xl_dot = cos(fl);
    yl_dot = sin(fl);
    fl_dot = 0;

    dydt = [xl_dot; yl_dot; fl_dot];
end

function dydt = ode_equations(~, y, k_1, k_2, rho_d, f_d, v_l, w_l, L, L_1)
    z1 = y(1);
    z2 = y(2);
    fe = y(3);

    v_f = v_l*cos(fe) + k_1*z1 + rho_d*w_l*sin(f_d + fe) - L_1*w_l*sin(fe - 1.571);
    w_f = (v_l*sin(fe) - k_2*z2 + L_1*w_l*cos(fe - 1.571))/L;

    z1_dot = -v_f + v_l*cos(fe) - z2*w_f - L_1*w_l*sin(fe - 1.571);
    z2_dot = -v_l*sin(fe) + z1*w_f + L*w_f - L_1*w_l*cos(fe - 1.571);
    fe_dot = -w_f + w_l;

    dydt = [z1_dot; z2_dot; fe_dot];
end
