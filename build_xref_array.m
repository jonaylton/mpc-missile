clear
clc
close all

load('missile_data.mat')

%% Solve Nonlinear System of Equations

% Define states (optimization variables)
x = optimvar('x',4); % alpha,M,q,delta

alpha = x(1);
M = x(2);
q = x(3);
delta = x(4);

% Initial height
h = 6096;

% Speed of sound equation for 0m <= h <= 11000m (Troposphere)
Vs = sqrt((T0-L*h)*gam*R);

% Air density equation for 0m <= h <= 11000m (Troposphere)
rho = rho0*(1-(L/T0)*h)^(g/(L*R)-1);

% Dynamic Pressure series
sigma = (1 + M^2/4 + M^4/40 + M^6/2100);

% Define aerodynamic coefficient equations as function of state variables
Cx = aa;
Cz = an*alpha^3 + bn*alpha*sqrt(alpha^2) + cn*(2-M/3)*alpha + ...
    dn*delta;
Cm = am*alpha^3 + bm*alpha*sqrt(alpha^2) + cm*(-7+8*M/3)*alpha + ...
    dm*delta + em*q;

% Aproximate Non-linear Dynamic
alpha_dot = (1/(2*mass))*rho*Vs*M*sigma*S_ref*Cz + q;

q_dot = (1/(2*Iyy))*rho*Vs^2*M^2*sigma*S_ref*d_ref*Cm;

% Define output
az = (1/(2*mass))*rho*Vs^2*M^2*sigma*S_ref*Cz;

% Problem setup
prob = eqnproblem;

% System of Equations
eq1 = alpha_dot == 0; % w = cte -> w_dot == 0 -> alpha_dot = 0
eq2 = Cm == 0; % q = cte -> q_dot == 0 -> Cm == 0

prob.Equations.eq1 = eq1;
prob.Equations.eq2 = eq2;

options = optimoptions(prob);
options.Display = "off";
options.Algorithm = "levenberg-marquardt";
% options.MaxIterations   = 1000;
% options.MaxFunctionEvaluations = 1000;
% options.FunctionTolerance = 0.001;

az_vec = 0:10:350; % n=36
az_vec = az_vec + 1e-6;
M_vec = 1.5:0.1:4.5; % n=31

proc_time = zeros(length(az_vec),length(M_vec));
xref = struct('x', repmat({zeros(length(x),1)}, length(M_vec), 1));
fval = struct('eq1', repmat({zeros(1)}, length(M_vec), 1), ...
              'eq2', repmat({zeros(1)}, length(M_vec), 1), ...
              'eq3', repmat({zeros(1)}, length(M_vec), 1), ...
              'eq4', repmat({zeros(1)}, length(M_vec), 1));
xref_mat = zeros(length(x),length(az_vec),length(M_vec));
fval_mat = zeros(4,length(az_vec),length(M_vec));

tic
for j = 1:length(az_vec)
    eq3 = az == az_vec(j);
    prob.Equations.eq3 = eq3;

    x0.x = [-1e-6 M_vec(1) -1e-6 1e-6]; % Initial guess

    for i=1:length(M_vec)
        t1 = tic;
        eq4 = M == M_vec(i);
        prob.Equations.eq4 = eq4;
        [xref(i),fval(i),~] = solve(prob,x0,Options=options);
        xref_mat(:,j,i)=xref(i).x; % xf_mat = sol = [alpha; M; q, delta]
        fval_mat(:,j,i)=struct2array(fval(i));
        x0.x = xref(i).x; % Initialize the next guess from previous solution
        proc_time(j,i) = toc(t1);
    end
end
toc

%% The next code is to check if y = h(xref) == yref

az_mat = zeros(length(az_vec),length(M_vec));

for j=1:length(az_vec)
    for i=1:length(M_vec)
        x = xref_mat(:,j,i);

        % Inputs
        alpha = x(1);
        M = x(2);
        q = x(3);
        delta = x(4);
    
        % Initial height
        h = 6096;
        
        % Speed of sound equation for 0m <= h <= 11000m (Troposphere)
        Vs = sqrt((T0-L*h)*gam*R);
        
        % Air density equation for 0m <= h <= 11000m (Troposphere)
        rho = rho0*(1-(L/T0)*h)^(g/(L*R)-1);

        % Dynamic Pressure series
        sigma = (1 + M^2/4 + M^4/40 + M^6/2100);
        
        % Define aerodynamic coefficient equations as function of state variables
        Cx = aa;
        Cz = an*alpha^3 + bn*alpha*sqrt(alpha^2) + cn*(2-M/3)*alpha + ...
            dn*delta;
        Cm = am*alpha^3 + bm*alpha*sqrt(alpha^2) + cm*(-7+8*M/3)*alpha + ...
            dm*delta + em*q;
        
        % Define output
        az_mat(j,i) = (1/(2*mass))*rho*Vs^2*M^2*sigma*S_ref*Cz;
    end
end

save('xref.mat','xref_mat')