function q_res = hw2_uzakotim()
    % Totally randomly chosen set of required end effector positions for which
    % you should solve the inverse kinematics problem
    x_des = [-1.16774193548387;-0.767741935483871;-0.967741935483871;-0.903225806451613;-1.14193548387097;-0.683870967741936;-0.187096774193548;-0.232258064516129;-0.425806451612904;-0.554838709677419;-0.593548387096775;-0.509677419354839;-0.212903225806451;0.0903225806451606;0.161290322580645;0.129032258064516;0.0129032258064515;-0.174193548387097;0.032258064516129;0.135483870967743;0.406451612903226;0.354838709677419;0.367741935483871;0.496774193548387;0.606451612903226;0.574193548387097;0.393548387096775;0.845161290322581;0.748387096774193;0.72258064516129;0.748387096774193;0.909677419354839;0.767741935483871;0.941935483870967;0.974193548387097;0.845161290322581;1.1741935483871;1.14838709677419;1.11612903225806;1.12258064516129;1.31612903225807;1.18709677419355;1.27741935483871;1.30967741935483;1.10322580645161];
    y_des = [0.353061224489796;0.3;0.336734693877551;0.806122448979592;0.797959183673469;0.80204081632653;0.671428571428572;0.777551020408163;0.797959183673469;0.728571428571428;0.585714285714285;0.459183673469388;0.275510204081633;0.451020408163265;0.60204081632653;0.744897959183673;0.797959183673469;0.679591836734693;0.793877551020408;0.740816326530612;0.736734693877552;0.60204081632653;0.406122448979591;0.295918367346938;0.406122448979591;0.622448979591837;0.736734693877552;0.761224489795918;0.66734693877551;0.320408163265306;0.479591836734693;0.33265306122449;0.487755102040816;0.557142857142857;0.671428571428572;0.724489795918367;0.724489795918367;0.659183673469388;0.336734693877551;0.495918367346938;0.369387755102041;0.520408163265306;0.540816326530612;0.663265306122448;0.728571428571428];

    % You can use a different guess for the initial conditions
    q_init = [2.8;0.1;0.7;2.6];

    % Initialize the variable storing the solutions of the inverse kinematics
    q_res = zeros(4, numel(x_des));

    % Solve the inverse kinematics problem for each required end effector
    % position
    for i = 1:numel(x_des)
        % Current required end effector position
        end_req = [x_des(i); y_des(i)];

        % Solve the inverse kinematics problem for end_req by BFGS
        q_opt = bfgs(@(th) objective_function(th, end_req), q_init);

        % Plot the robot for the optimal joint coordinates q_opt
        figure(1)
        plot([-0.5 0.5], [0 0], 'k', 'LineWidth', 2)
        hold on
        plot(x_des(i), y_des(i), 'rx')
        plotRobot(q_opt)
        hold off
        grid on
        xlim([0 1.5])
        ylim([-1.5 1.5])
        axis equal    
        legend('Required end effector position')

        pause(0.1)

        % Store the optimal joint coordinates to q_res and...
        q_res(:, i) = q_opt;
        % ... use them as the initial guess for the next iteration
        q_init = q_opt;
    end

end

function q_pos = joints_pos(q)
% joints_pos() - function returning positions of all joints of
% the planar 4-link robot
%
% inputs:
%    q - 4-element vector of joint coordinates (mutual angles between individual joints)
% outputs:
%    q_pos - positions of robot's joints in plane 

    q_pos = reshape([0.0,cos(q(1)).*(3.0./1.0e1),cos(q(1)+q(2)).*(2.0./5.0)+cos(q(1)).*(3.0./1.0e1),cos(q(1)+q(2)+q(3))./2.0+cos(q(1)+q(2)).*(2.0./5.0)+cos(q(1)).*(3.0./1.0e1),cos(q(1)+q(2)+q(3))./2.0+cos(q(1)+q(2)+q(3)+q(4)).*(3.0./1.0e1)+cos(q(1)+q(2)).*(2.0./5.0)+cos(q(1)).*(3.0./1.0e1),0.0,sin(q(1)).*(3.0./1.0e1),sin(q(1)+q(2)).*(2.0./5.0)+sin(q(1)).*(3.0./1.0e1),sin(q(1)+q(2)+q(3))./2.0+sin(q(1)+q(2)).*(2.0./5.0)+sin(q(1)).*(3.0./1.0e1),sin(q(1)+q(2)+q(3))./2.0+sin(q(1)+q(2)+q(3)+q(4)).*(3.0./1.0e1)+sin(q(1)+q(2)).*(2.0./5.0)+sin(q(1)).*(3.0./1.0e1)],[5,2]);

end


function end_pos = endeffector_pos(q)
% endeffector_pos() - function returning position of the end effector of
% the planar 4-link robot
%
% inputs:
%    q - 4-element vector of joint coordinates (mutual angles between individual joints)
% outputs:
%    end_pos - position of robot's end effector in plane 

    jp = joints_pos(q);
    end_pos = jp(end,:);

end

function cost = objective_function(q, req_pos)
% objective_function() - Objective function for the inverse kinematics
% problem for the planar 4-link robot
%
% inputs:
%    q - 4-element vector of joint coordinates (mutual angles between individual joints)
%    req_pos - 2-element vector of required end effector position
% outputs:
%    cost - the value of the objective function

    end_pos = endeffector_pos(q);
    cost = (end_pos(1) - req_pos(1))^2 + (end_pos(2) - req_pos(2))^2;
end

function plotRobot(q)
    jp = joints_pos(q);
    
    plot(jp(:,1), jp(:,2), '-o');
end

function x_res = bfgs(f, x0)
% bfgs() - Broyden–Fletcher–Goldfarb–Shanno (BFGS) algorithm
%
% inputs:
%    f - handler of an objective function to be optimized
%    x0 - initial guess for the optimal decision vector
% outputs:
%    x_res - the found optimal decision vector

    epsilon = 1e-5;
    maxiter = 2000;

    n = numel(x0);
    x = x0;

    % Initialize BFGS algorithm here
    B0 = eye(size(x0,1));
    Bk = B0;
    iter=0;
    xk = x;
    s = 2;
    gamma = 1/4;
    beta = 1/2;
    grad = numGrad(xk,f);
    while (norm(grad)>epsilon) && iter < maxiter
        iter = iter+1;
        % Implement the rest of BFGS algorithm here (use numGrad() for computing the gradient of f)
        % Obtain direction pk
        pk = -Bk\numGrad(xk,f);
        
        % Line search
        % Backtracking
        fun_val = f(xk);
        alph = s;
        while (fun_val-f(xk-alph * numGrad(xk,f)) < gamma * alph * norm(numGrad(xk,f))^2)
                alph = beta * alph;
        end
        
        % Set sk
        sk = alph*pk;
        % Store xk and update it with sk
        x_prev = xk;
        xk = xk + sk;
        % Update Hessian aprx Hk
        yk = numGrad(xk,f)-numGrad(x_prev,f);
        Bk = Bk + (yk*yk')/(yk'*sk) - ((Bk*sk)*sk'*Bk')/(sk'*Bk*sk);
        % Update grad
        grad = numGrad(xk,f);
        
        fprintf('iter_number = %3d norm_grad = %2.6f fun_val = %2.6f \n',...
            iter,norm(grad),fun_val);

    end

    x_res = xk;
end

function gradval = numGrad(x, f)
% numGrad() - Numerical computation of gradient
%
% inputs:
%    x - argument of f() for which the gradient is computed
%    f - handler to a function for which the gradient is computed
% outputs:
%    gradval - value of the numerically computed gradient

    d = 1e-4;
    gradval = zeros(size(x));
    perturb = zeros(size(x));
    for i=1:numel(gradval)
        perturb(i) = d;
        gradval(i) = (f(x+perturb) - f(x-perturb))/2/d;
        perturb(i) = 0;
    end
end