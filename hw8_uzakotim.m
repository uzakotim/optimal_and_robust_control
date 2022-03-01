function [ t_star, x_star, u_star, Tf_star ] = hw8_uzakotim()
    m = 1; % mass of the pendulum
    l = 1; % length of the pendulum
    g = 9.81; % gravitational acceleration
    b = 0.1; % friction coefficient

    a1 = g/l;
    a2 = b/(m*l*l);
    a3 = 1/(m*l*l);

    % Parameters
    N = 50;

    umin = -2.75;
    umax = 2.75;

    % Initialize the optimization problem
    opti = casadi.Opti();

    % define decision variables
    X = opti.variable(2,N+1);

    th = X(1, :);
    Dth = X(2, :);
    U = opti.variable(1,N+1);
    Tf = opti.variable();

    % Objective function (minimize the control effort)

    opti.minimize(U*U');
    opti.minimize(Tf);

    % Dynamic constraints
    f = @(x,u) pend_ode(x, u, a1, a2, a3); % dx/dt = f(x,u)
    Ts = Tf/N; % control interval length


    % for k=1:N % loop over control intervals
    %             % Runge-Kutta 4 integration
    %             k1 = f(X(:,k),         U(:,k));
    %             k2 = f(X(:,k)+Ts/2*k1, U(:,k));
    %             k3 = f(X(:,k)+Ts/2*k2, U(:,k));
    %             k4 = f(X(:,k)+Ts*k3,   U(:,k));
    %             x_next = X(:,k) + Ts/6*(k1+2*k2+2*k3+k4);
    %             opti.subject_to(X(:,k+1)==x_next); % close the gaps
    %         end



     for k=1:N % loop over control intervals
         k1 = f(X(:,k),U(:,k));
         k2 = f(X(:,k+1),U(:,k+1));
         Xc = 0.5*(X(:,k)+X(:,k+1))+(Ts/8)*(k1-k2);
         Uc = (U(:,k)+U(:,k+1))/2;
         k3 = f(Xc,Uc);
         x_next = X(:,k) + (Ts/6)*(k1+4*k3+k2);
         opti.subject_to(X(:,k+1)==x_next);
          % close the gaps
     end

    opti.subject_to(umin<=U<=umax);
    opti.subject_to(Tf>=0);

    % Initial conditions
    opti.subject_to(th(1)==0);   % start at position 0 ...
    opti.subject_to(Dth(1)==0); % ... from stand-still 

    % Final-time constraints
    opti.subject_to(th(end)==pi);
    opti.subject_to(Dth(end)==0);

    % Initialize decision variables
    opti.set_initial(U, 0);
    opti.set_initial(th, 0);
    opti.set_initial(Dth, 0);
    opti.set_initial(Tf, 1);



    % Solve NLP
    opti.solver('ipopt'); % use IPOPT solver
    sol = opti.solve();

    % Extract the states and control from the decision variables
    x_star = sol.value(X)';
    u_star = sol.value(U)';
    Tf_star = sol.value(Tf);
    dt = Tf_star/N;
    t_star = (0:N)*dt;

    figure(1)
    subplot(211)
    hold on
    plot(t_star, x_star, '--');
    hold off
    grid on
    xlabel('Time [s]')
    ylabel('States [-]')
    legend('trajectory','derivative')

    subplot(212)
    hold on
    plot(t_star(1:end), u_star, 'r--');
    hold off
    grid on
    xlabel('Time [s]')
    ylabel('Controls [Nm]')
    legend('NLP')

    disp("minimal time:")
    disp(Tf_star)
end

function dx = pend_ode(x, u, a1, a2, a3)
% x(1) - theta
% x(2) - Dtheta

dx = [x(2);-a2*x(2) - a1*sin(x(1)) + a3*u];

end