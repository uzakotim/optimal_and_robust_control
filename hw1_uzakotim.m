function [optval, opt_X, opt_type] = hw1_uzakotim()
N = 10; % # hours in the plan
m = 3;  % # cars

Emax = [11.6;11.9;10.6;8.8;8;8.8;10.6;11.9;11.6;10]; % Maximum available energy per hour [kWh]
cost = [0.58;0.72;0.92;0.68;0.54;0.78;0.64;0.57;0.74;0.74]; % Cost of the energy [€/kWh]

Ereqk = [15;25;30]; % Minimum requested charged energy for individual cars
Emaxk = [6;6;4];    % Maximum allowed energy you can charge to individual cars per hour
dk = [3;7;10];      % Departure times of the cars

ek = zeros(N, m);

cvx_begin
    variable x(N,m)
    
    minimize( sum(cost'*x) )
    subject to
    % terms must be positive    
        x >= ek
    % available energy per hour constraint: sum of row elements of x
        sum(x,2)<= Emax
    % required energy till departure: sum of dk(k) cells for each car
        for k = 1:m
        sum(x(1:dk(k),k))>=Ereqk(k)
    % each hour charging compared to max allowed energy per hour for each
    % k-th car
        x(:,k)<= Emaxk(k)
        end
cvx_end

optval = sum(cost'*x);
opt_X = x;
opt_type = 'LP';
%disp(x)
%disp(opt_type)
end