close all
clear
clc

load('missile_data.mat')

Thrust = 0;   % Post-burnout thrust
% em = 0;     % Dumping derivative
% g_dyn = 0;  % Gravity
g_dyn = g;    % Gravity

%% Definitions of variables (States, Inputs and Outputs)

import casadi.*

% Define state variables
alpha = SX.sym('alpha');
M = SX.sym('M');
q = SX.sym('q');
h = SX.sym('h');
theta = SX.sym('theta');
delta = SX.sym('delta');
delta_dot = SX.sym('delta_dot');

x = [alpha; M; q; h; theta; delta; delta_dot];
nx = length(x);

% Speed of sound equation for 0m <= h <= 11000m (Troposphere)
Vs = sqrt((T0-L*h)*gam*R);

% Air density equation for 0m <= h <= 11000m (Troposphere)
rho = rho0*(1-(L/T0)*h)^(g/(R*L)-1);

% Dynamic Pressure series
sigma = (1 + M^2/4 + M^4/40 + M^6/2100);
% sigma = 1; % Rough approximation

% Define parameters as optim variables
aa_par = SX.sym('aa_par');
an_par = SX.sym('an_par');
bn_par = SX.sym('bn_par');
cn_par = SX.sym('cn_par');
dn_par = SX.sym('dn_par');
am_par = SX.sym('am_par');
bm_par = SX.sym('bm_par');
cm_par = SX.sym('cm_par');
dm_par = SX.sym('dm_par');
em_par = SX.sym('em_par');

p = [aa_par; an_par; bn_par; cn_par; dn_par; am_par; bm_par; cm_par; dm_par; em_par];
np = length(p);

% Define aerodynamic coefficient equations as function of state variables
Cx = aa_par;
Cz = an_par*alpha^3 + bn_par*alpha*abs(alpha) + cn_par*(2-M/3)*alpha + ...
    dn_par*delta;
Cm = am_par*alpha^3 + bm_par*alpha*abs(alpha) + cm_par*(-7+8*M/3)*alpha + ...
    dm_par*delta + em_par*q;

% Define input variables
T = SX.sym('T');
delta_c = SX.sym('delta_c');

u = [T; delta_c];
nu = length(u);

% Define output variables
az = (1/(2*mass))*rho*Vs^2*M^2*sigma*S_ref*Cz;
gamma = theta-alpha;

y = [alpha; M; q; az; gamma];
ny = length(y);

% Define disturbance and uncertainty variables
w = SX.sym('w',nx);
v = SX.sym('v',ny);

%% MPC internal model (used for simulation and to update internal model states)

alpha_dot = (1/(2*mass))*rho*Vs*M*sigma*S_ref*(-Cx*sin(alpha)+Cz*cos(alpha)) + ...
   (1/(Vs*M))*g_dyn*cos(gamma) + q - (1/(mass*Vs*M))*T*sin(alpha);

M_dot = (1/(2*mass))*rho*Vs*M^2*sigma*S_ref*(Cx*cos(alpha)+Cz*sin(alpha)) - ...
    (1/Vs)*g_dyn*sin(gamma) + (1/(mass*Vs))*T*cos(alpha);

q_dot = (1/(2*Iyy))*rho*Vs^2*M^2*sigma*S_ref*d_ref*Cm;

h_dot = M*Vs*sin(theta-alpha);
% h_dot = 0;

theta_dot = q;

delta_dot_dot = -wn_fin^2*delta -2*z_fin*wn_fin*delta_dot + ...
    wn_fin^2*delta_c;

x_dot = [alpha_dot; M_dot; q_dot; h_dot; theta_dot; delta_dot; delta_dot_dot];

% State Derivatives Function
f_dx = Function('f_dx',{x,u,p},{x_dot},{'x','u','p'},{'x_dot'});

% Output Function
f_y = Function('f_y',{x,p},{y},{'x','p'},{'y'});

%% Construct integrator using Runge-Kutta

Ts = 0.02; % Sample Time

% Integrator to discretize the system
intg_opts = struct;
intg_opts.simplify = true;
intg_opts.number_of_finite_elements = 4; % Order of the method

% DAE problem structure
dae = struct;
dae.x = x; % States
dae.u = [u;p]; % Inputs
ode = f_dx(x,u,p);
dae.ode = ode; % Expression for the right hand side

intg = integrator('intg','rk',dae,0,Ts,intg_opts);

rk = intg('x0',x,'u',[u;p]); % Evaluate with symbols

x_nxt = rk.xf;

% Function to get next state using Runge-Kutta with sample time = Ts
f_x_nxt = Function('f_x_nxt',{x,u,p},{x_nxt},{'x','u','p'},{'x_nxt'});

%% Linearization

% Define state linearization point
alpha_lin = SX.sym('alpha_lin');
M_lin = SX.sym('M_lin');
q_lin = SX.sym('q_lin');
h_lin = SX.sym('h_lin');
theta_lin = SX.sym('theta_lin');
delta_lin = SX.sym('delta_lin');
delta_dot_lin = SX.sym('delta_dot_lin');

x_lin = [alpha_lin; M_lin; q_lin; h_lin; theta_lin; delta_lin; delta_dot_lin];

% State Update Linearized
x_nxt_lin = linearize(x_nxt,x,x_lin);

f_x_nxt_lin = Function('f_x_nxt_lin',{x_lin,x,u,p},{x_nxt_lin},{'x_lin','x','u','p'},{'x_nxt_lin'});

% Output Function Linearized
y_lin = linearize(y,x,x_lin);

f_y_lin = Function('f_y_lin',{x_lin,x,p},{y_lin},{'x_lin','x','p'},{'y_lin'});

% Jacobians Matrices
A = jacobian(x_nxt,x);
B = jacobian(x_nxt,u);
C = jacobian(y,x);

% Function to evaluate jacobians
f_A = Function('f_A',{x,u,p},{A},{'x','u','p'},{'A'});
f_B = Function('f_B',{x,u,p},{B},{'x','u','p'},{'B'});
f_C = Function('f_C',{x,u,p},{C},{'x','u','p'},{'C'});

%% MPC internal model struct

model_full = struct();
model_full.Ts = Ts;
model_full.x = x;
model_full.nx = nx;
model_full.u = u;
model_full.nu = nu;
model_full.p = p;
model_full.np = np;
model_full.y = y;
model_full.ny = ny;
model_full.x_dot = x_dot;
model_full.x_nxt = x_nxt;
model_full.x_lin = x_lin;
model_full.x_nxt_lin = x_nxt_lin;
model_full.y_lin = y_lin;
model_full.f_dx = f_dx;
model_full.f_y = f_y;
model_full.f_x_nxt = f_x_nxt;
model_full.f_x_nxt_lin = f_x_nxt_lin;
model_full.f_y_lin = f_y_lin;
model_full.A = A;
model_full.B = B;
model_full.C = C;
model_full.f_A = f_A;
model_full.f_B = f_B;
model_full.f_C = f_C;

%% MPC simplified model (used for optimization, used only in the horizon window)

% Define system dynamics
alpha_dot = (1/(2*mass))*rho*Vs*M*sigma*S_ref*Cz + q;

M_dot = (1/(2*mass))*rho*Vs*M^2*sigma*S_ref*Cx;

q_dot = (1/(2*Iyy))*rho*Vs^2*M^2*sigma*S_ref*d_ref*Cm;

h_dot = 0;

theta_dot = 0;

delta_dot_dot = -wn_fin^2*delta -2*z_fin*wn_fin*delta_dot + ...
    wn_fin^2*delta_c;

x_dot = [alpha_dot; M_dot; q_dot; h_dot; theta_dot; delta_dot; delta_dot_dot];

% State Derivatives Function
f_dx = Function('f_dx',{x,u,p},{x_dot},{'x','u','p'},{'x_dot'});

%% Construct integrator using Runge-Kutta

Ts = 0.02; % Sample Time

% Integrator to discretize the system
intg_opts = struct;
intg_opts.simplify = true;
intg_opts.number_of_finite_elements = 4; % Order of the method

% DAE problem structure
dae = struct;
dae.x = x; % States
dae.u = [u;p]; % Inputs
ode = f_dx(x,u,p);
dae.ode = ode; % Expression for the right hand side

intg = integrator('intg','rk',dae,0,Ts,intg_opts);

rk = intg('x0',x,'u',[u;p]); % Evaluate with symbols
x_nxt = rk.xf;

% Function to get next state using Runge-Kutta with sample time = Ts
f_x_nxt = Function('f_x_nxt',{x,u,p},{x_nxt},{'x','u','p'},{'x_nxt'});

%% Linearization

% State Update Linearized
x_nxt_lin = linearize(x_nxt,x,x_lin);

f_x_nxt_lin = Function('f_x_nxt_lin',{x_lin,x,u,p},{x_nxt_lin},{'x_lin','x','u','p'},{'x_nxt_lin'});

% Output Function Linearized
y_lin = linearize(y,x,x_lin);

f_y_lin = Function('f_y_lin',{x_lin,x,p},{y_lin},{'x_lin','x','p'},{'y_lin'});

% Jacobians Matrices
A = jacobian(x_nxt,x);
B = jacobian(x_nxt,u);
C = jacobian(y,x);

% Function to evaluate jacobians
f_A = Function('f_A',{x,u,p},{A},{'x','u','p'},{'A'});
f_B = Function('f_B',{x,u,p},{B},{'x','u','p'},{'B'});
f_C = Function('f_C',{x,u,p},{C},{'x','u','p'},{'C'});

%% MPC internal plant struct (used for optimization)

model = struct();
model.Ts = Ts;
model.x = x;
model.nx = nx;
model.u = u;
model.nu = nu;
model.p = p;
model.np = np;
model.y = y;
model.ny = ny;
model.x_dot = x_dot;
model.x_nxt = x_nxt;
model.x_lin = x_lin;
model.x_nxt_lin = x_nxt_lin;
model.y_lin = y_lin;
model.f_dx = f_dx;
model.f_y = f_y;
model.f_x_nxt = f_x_nxt;
model.f_x_nxt_lin = f_x_nxt_lin;
model.f_y_lin = f_y_lin;
model.A = A;
model.B = B;
model.C = C;
model.f_A = f_A;
model.f_B = f_B;
model.f_C = f_C;

%% Multiple Shooting MPC controller

Np = 8; % Prediction horizon
Nc = 2; % Control horizon

mpc = Opti(); % Optimization problem

% Decision variables (symbolic)
X_sym = mpc.variable(nx, Np+1); % States over prediction horizon + x0
U_sym = mpc.variable(nu, Np); % Controls over prediction horizon
Y_sym = mpc.variable(ny, 1); % Final predicted output (using mpc model)

% Parameters (symbolic)
x0_sym = mpc.parameter(nx, 1);  % Initial state
p0_sym = mpc.parameter(np, 1);  % Most recent parameters estimation
yref_sym = mpc.parameter(1, 1); % Target output
T_sym = mpc.parameter(1, Np);   % Thrust over prediction horizon (known 
% function of time)

% Weigthing matrix (For scaling, feasibility is very sensitive due to 
% matrix condition number)
P = 1e0;
Q = 1e2*diag([0 0 1 0 0 1 1]);
Ru = 0e0;

% Cost function initialization
mpc_cost = 0;

% Initial state constraint
mpc.subject_to(X_sym(:,1) == x0_sym);

for k = 1:Np
    % Cost due to state difference
    mpc_cost = mpc_cost + (X_sym(:,k) - X_sym(:,Np+1))'*Q*(X_sym(:,k) - X_sym(:,Np+1));

    % Cost due to input difference
    mpc_cost = mpc_cost + (U_sym(2,k) - X_sym(6,Np+1))'*Ru*(U_sym(2,k) - X_sym(6,Np+1)); % X(6) = delta

    % Gap closing constraint
    mpc.subject_to(X_sym(:,k+1) == model.f_x_nxt(X_sym(:,k), U_sym(:,k), p0_sym)); % rk integration
    % mpc.subject_to(X_sym(:,k+1) == model.f_x_nxt_lin(x0_sym, (X_sym(:,k) - x0_sym), U_sym(:,k), p0_sym)); % rk integration
end

% Output at the end of the prediction horizon
% (Terminal equality constraint to garantee stability)
mpc.subject_to(Y_sym(:,1) == model.f_y(X_sym(:,Np+1), p0_sym));

% Cost due to output at the end of the prediction horizon
mpc_cost = mpc_cost + (Y_sym(4,1) - yref_sym)'*P*(Y_sym(4,1) - yref_sym);

% Physical constraints
mpc.subject_to(-fin_maxrate <= X_sym(7,1:Nc) <= fin_maxrate); % delta_dot limit (rk4)

mpc.subject_to(U_sym(1,:) == T_sym(1,:)); % Thrust reference

if Nc < Np % Control horizon
    % Input 2 (fin deflection) is repeated from nc to np
    mpc.subject_to(U_sym(2,Nc+1:end) == repmat(U_sym(2,Nc),1,Np-Nc));
end

mpc.minimize(mpc_cost);

% Function for rapid cost evaluation
f_mpc_cost = Function('f_mpc_cost',{X_sym,U_sym,Y_sym,x0_sym,p0_sym,yref_sym,T_sym},{mpc_cost},{'X_sym','U_sym','Y_sym','x0_sym','p0_sym','yref_sym','T_sym'},{'mpc_cost'});

%% Setup solver
% JIT
jit_options = struct('compiler', 'cl.exe','verbose',false);
% opti.solver('sqpmethod',struct('jit',true,'compiler','shell',...
%     'jit_options',jit_options)); % example

% QRQP
qpsol_options = struct('print_iter',false,'print_header',false,...
    'error_on_fail',false,'verbose',false,'max_iter',7);
sqp_options = struct('qpsol','qrqp','qpsol_options',qpsol_options, ...
    'print_iteration',false,'print_header',false,'print_status',false,...
    'print_time',false,'tol_pr',1e-4,'tol_du',1e-2,'max_iter',7,...
    'error_on_fail',false,'hessian_approximation','exact',...
    'jit',true,'compiler','shell','jit_options',jit_options);
mpc.solver('sqpmethod',sqp_options);

% OSQP
% qpsol_options = struct('error_on_fail',false);
% sqp_options = struct('qpsol','osqp','qpsol_options',qpsol_options,...
%     'print_iteration',false,'print_header',false,...
%     'print_status',false,'print_time',false,...
%     'tol_pr',1e-4,'tol_du',1e-2,'max_iter',7,...
%     'error_on_fail',false,'hessian_approximation','exact');
% mpc.solver('sqpmethod',sqp_options);

% IPOPT-1
% ipopt_options = struct('ipopt',struct('print_level',0,'tol',10, ...
%     'fast_step_computation','yes','max_cpu_time',Ts,'max_iter', 4), ...
%     'print_time', false, 'jit',true,'compiler','shell',...
%     'jit_options',jit_options);
% mpc.solver('ipopt', ipopt_options, struct('max_iter', 4, ...
%     'max_cpu_time', Ts, 'print_level', 0));

% IPOPT-2
% ipopt_options = struct('ipopt',struct('print_level',0,...
%     'fast_step_computation','yes','max_cpu_time',Ts), ...
%     'print_time', false, 'jit',true,'compiler','shell',...
%     'jit_options',jit_options);
% mpc.solver('ipopt', ipopt_options, struct(...
%     'max_cpu_time', Ts, 'print_level', 0));

%% Multiple Shooting MHE estimator

Ne = 8; % Estimation horizon

mhe = Opti(); % Optimization problem

% Decision variables (symbolic)
Xe_sym = mhe.variable(nx, Ne+1); % States over estimation horizon + 1
Ue_sym = mhe.variable(nu, Ne);   % Inputs over estimation horizon
Pe_sym = mhe.variable(np, 1);    % Parameter estimation as decision variables
Ye_sym = mhe.variable(ny, Ne+1); % Outputs over estimation horizon + 1

% Parameters (symbolic)
xk_sym = mhe.parameter(nx, Ne+1); % Most recent Ne+1 internal states
uk_sym = mhe.parameter(nu, Ne);   % Most recent Ne measured inputs
yk_sym = mhe.parameter(1, Ne+1);  % Most recent Ne+1 measured outputs

% Weigthing matrix (For scaling, feasibility is very sensitive due to 
% matrix condition number)
Q = eye(1);

% Cost function initialization
mhe_cost = 0;

mhe.subject_to(Pe_sym(1,1) == aa)
% mhe.subject_to(Pe_sym(2,1) == an)
% mhe.subject_to(Pe_sym(3,1) == bn)
% mhe.subject_to(Pe_sym(4,1) == cn)
% mhe.subject_to(Pe_sym(5,1) == dn)
mhe.subject_to(Pe_sym(6,1) == am)
mhe.subject_to(Pe_sym(7,1) == bm)
mhe.subject_to(Pe_sym(8,1) == cm)
mhe.subject_to(Pe_sym(9,1) == dm)
mhe.subject_to(Pe_sym(10,1) == em)

% mhe.subject_to(abs(aa)*0.9 < abs(Pe_sym(1,1)) < abs(aa)*1.1)
mhe.subject_to(abs(an)*0.9 < abs(Pe_sym(2,1)) < abs(an)*1.1)
mhe.subject_to(abs(bn)*0.9 < abs(Pe_sym(3,1)) < abs(bn)*1.1)
mhe.subject_to(abs(cn)*0.9 < abs(Pe_sym(4,1)) < abs(cn)*1.1)
mhe.subject_to(abs(dn)*0.9 < abs(Pe_sym(5,1)) < abs(dn)*1.1)
% mhe.subject_to(abs(am)*0.9 < abs(Pe_sym(6,1)) < abs(am)*1.1)
% mhe.subject_to(abs(bm)*0.9 < abs(Pe_sym(7,1)) < abs(bm)*1.1)
% mhe.subject_to(abs(cm)*0.9 < abs(Pe_sym(8,1)) < abs(cm)*1.1)
% mhe.subject_to(abs(dm)*0.9 < abs(Pe_sym(9,1)) < abs(dm)*1.1)
% mhe.subject_to(abs(em)*0.9 < abs(Pe_sym(10,1)) < abs(em)*1.1)

for k = 1:Ne+1
    % Cost due to output difference
    mhe_cost = mhe_cost + (Ye_sym(4,k) - yk_sym(:,k))'*Q*(Ye_sym(4,k) - yk_sym(:,k));

    if k <= Ne
        % Input data equality constraint
        mhe.subject_to(Ue_sym(:,k) == uk_sym(:,k))

        % Gap closing equality constraint
        mhe.subject_to(Xe_sym(:,k+1) == model.f_x_nxt(Xe_sym(:,k), Ue_sym(:,k), Pe_sym(:,1))); % rk integration
    end

    % Output equation equality constraint
    mhe.subject_to(Ye_sym(:,k) == model.f_y(Xe_sym(:,k), Pe_sym(:,1)))
end

% Physical constraints
mhe.subject_to(-50*d2r <= Xe_sym(7,1:Ne+1) <= 50*d2r) % delta_dot limit (rk4)

mhe.minimize(mhe_cost);

%% Setup solver

% IPOPT
ipopt_options = struct('ipopt',struct('print_level',0,'tol',10, ...
    'fast_step_computation','yes','max_cpu_time',Ts,'max_iter',4), ...
    'print_time', false, 'jit',true,'compiler','shell',...
    'jit_options',jit_options);
mhe.solver('ipopt', ipopt_options, struct('max_iter', 4, ...
    'max_cpu_time', Ts, 'print_level', 0));

%% Declare functions to solve and evaluate mpc and mhe optimization problem
mpc_eval = @(x0_, p0_, yref_) mpc_solve(mpc, x0_, x0_sym, p0_, p0_sym, yref_, yref_sym, U_sym);
mhe_eval = @(uk_, yk_) mhe_solve(mhe, uk_, uk_sym, yk_, yk_sym, Pe_sym);

%% Declare functions to plot V_N surface
V_N_plot_eval = @(x_0, x_r, u_r, p, y_r, N, P, Q, R) V_N_plot(N, x_0, x_r, u_r, p, y_r, N, P, Q, R, model_full);

%% Plot V_N for initial condition
% Initial condition
x0 = [alpha_ini;M_ini;0;h_ini;0;0;0];
p0 = [aa;an;bn;cn;dn;am;bm;cm;dm;em];

load('xref.mat')

y_r = 300;
M_r = 4;

az_idx = y_r/10 + 1;
M_idx = (M_r - 1.5)/0.1 + 1;

x_r = [xref_mat(1,az_idx,M_idx), xref_mat(2,az_idx,M_idx), ...
    xref_mat(3,az_idx,M_idx), h_ini, 0, xref_mat(4,az_idx,M_idx), 0]';

% Weigthing matrices
P = 1e-3;
Q = 1e2*diag([0 0 1 0 0 1 1]);
Ru = 0e0;

V_N_plot(x0, x_r, x_r(6), p0, y_r, Np, P, Q, Ru, model_full)

%% Simulate

% Initial condition
x0 = [alpha_ini;M_ini;0;h_ini;0;0;0];
x0_plant = x0;
u0 = [Thrust; 0];
pnom = [aa;an;bn;cn;dn;am;bm;cm;dm;em];
p0 = pnom;
p_plant = pnom;
y0 = full(model_full.f_y(x0,p0));

Tf = 10; % Final simulation time
t_vec = 0:Ts:Tf;

fref = 1; % Reference signal frequency [Hz] (solver feasibility is very sensitive to reference signal freq.)

% yref_vec = 15*sin(2*pi*fref*t_vec)*g; % Sinusoidal

% yref_vec = 15*ones(1,length(0:Ts:Tf))*g; % Step

yref_vec = [30*ones(1,length(0:Ts:1-Ts)) -5*ones(1,length(1:Ts:2-Ts)) ...
    -30*ones(1,length(2:Ts:3-Ts)) 0*ones(1,length(3:Ts:4-Ts)) ...
    30*ones(1,length(4:Ts:5-Ts)) 5*ones(1,length(5:Ts:6-Ts)) ...
    -15*ones(1,length(6:Ts:7-Ts)) -30*ones(1,length(7:Ts:8-Ts)) ...
    -5*ones(1,length(8:Ts:9-Ts)) 20*ones(1,length(9:Ts:10))]*g; % Step sequence

% yref_vec = [30*ones(1,length(0:Ts:1-Ts)) -15*ones(1,length(1:Ts:2-Ts)) ...
%     -10*ones(1,length(2:Ts:3-Ts)) 5*ones(1,length(3:Ts:4.6))]*g; % Step sequence

% Real plant data
x_traj = [x0 zeros(nx,length(yref_vec)-1)];
u_traj = [u0 zeros(nu,length(yref_vec)-1)];
p_traj = [p0 zeros(np,length(yref_vec)-1)];
y_traj = [y0 zeros(ny,length(yref_vec)-1)];

% MPC internal data
xmpc_traj = [x0 zeros(nx,length(yref_vec)-1)];
ympc_traj = [y0 zeros(ny,length(yref_vec)-1)];

% OBS.: When the problem is nonconvex it's always a good idea to provide 
% initial guesses for decision variables

% MPC initialization
mpc.set_value(T_sym,repmat(Thrust,1,Np));
mpc.set_initial(U_sym(1,:),mpc.value(T_sym(1,:)));
mpc.set_initial(U_sym(2,:),zeros(1,Np));
sim = model.f_x_nxt.mapaccum(Np);
mpc.set_initial(X_sym,[x0 sim(x0, [mpc.value(T_sym(1,1:end)); zeros(1,Np)], p0)]);

% MHE initialization
mhe.set_initial(Pe_sym,pnom);
pk_vec = [p0 zeros(np,length(yref_vec)-1)];
mhe_cost_vec = [1e3 zeros(1,length(yref_vec)-1)];

% Call solver once to reduce next call overhead
[~, uk] = mpc_solve(mpc, x0, x0_sym, p0, p0_sym, 0, yref_sym, U_sym);
[~, pk] = mhe_solve(mhe, u_traj(:,1:Ne), uk_sym, y_traj(4,1:Ne+1), yk_sym, Pe_sym);

% Insert disturbance
%
% x(k+1) = A*x(k) + B*u(k) + Bw*w(k)
%
Bw = 0*diag([1e-3 1e-2 1e-1 0 0 1e-2 1e-2]); % normalized

% Insert output noise
%
% y(k) = C*x(k) + D*u(k) + Dv*v(k)
%
Dv = 0*diag([0 0 0 1 0]); % only affects output 4 (az)

% Lx = [-0.0001; 0; -0.06; 0; 0; -0.004; 0];

% j=1;

proc_time = zeros(1,length(yref_vec)-1);

t1 = tic;
for k=1:length(yref_vec)-1
    v = 0*(2.*rand(5,1)-1)*1e1; % random sensor noise [-10; 10]

    yref = yref_vec(k) + v(4);

    t2 = tic;

    % Parameter Estimation Using Least Squares
    % if k > Ne
    %     [cost_pk, pk] = mhe_solve(mhe, u_traj(:,k-Ne:k-1), uk_sym, y_traj(4,k-Ne:k), yk_sym, Pe_sym);
    % 
    %     if cost_pk < mhe_cost_vec(j)
    %         mhe_cost_vec(j+1) = cost_pk;
    %         pk_vec(:,j+1) = pk;
    %         p0 = pk;
    %         j = j+1;
    %     end
    % end

    [~, uk] = mpc_solve(mpc, x0, x0_sym, p0, p0_sym, yref, yref_sym, U_sym);

    proc_time(k) = toc(t2);
    
    % Force fin_min <= uk(2) <= fin_max
    if uk(2) > fin_max
        uk(2) = fin_max;
    elseif uk(2) < fin_min
        uk(2) = fin_min;
    end

    u0 = uk;

    % Get plant measurement
    w = 0*(2.*rand(nx,1)-1); % random disturbance

    % Real plant dynamic
    xk_plant = model_full.f_x_nxt(x0_plant,uk,p_plant) + Bw*w; % plant dynamics + disturbance

    yk_plant = model_full.f_y(xk_plant,p_plant) + Dv*v; % plant output

    x0_plant = xk_plant;

    % MPC internal model dynamic
    xk_mpc = model_full.f_x_nxt(x0,uk,p0);

    yk_mpc = model_full.f_y(xk_mpc,p0);

    % Full state measurement/observer
    % x0 = xk_plant;
    x0 = xk_mpc;
    % x0 = xk_mpc + Lx*(yk_plant(4) - yk_mpc(4));
    
    % Real plant trajectory
    x_traj(:,k+1) = full(xk_plant);
    u_traj(:,k+1) = uk;
    p_traj(:,k+1) = full(p0);
    y_traj(:,k+1) = full(yk_plant);

    % MPC internal model trajectory
    xmpc_traj(:,k+1) = full(x0);
    ympc_traj(:,k+1) = full(yk_mpc);
end
t2 = toc(t1)

% Plot az
figure('Name','az')
plot(t_vec,y_traj(4,:)/g,'Color','k');
hold
plot(t_vec,ympc_traj(4,:)/g,'Color','r');
plot(t_vec,yref_vec/g,'LineWidth',1,'LineStyle','--','Color','r')
ylim([-40 40])
hh = title('Acelera\c{c}\~ao normal ($a_Z$)', 'Interpreter', 'latex');
set(hh, 'Interpreter', 'latex');
ylabel('Aceleração [g]')
xlabel('Tempo [s]')

% Plot err = y_plant - y_mpc
figure('Name','az error')
plot(t_vec,(y_traj(4,:) - ympc_traj(4,:))/10,'Color','k')
hold
hh = title('Erro ($y_{plant} - y_{mpc}$)');
set(hh, 'Interpreter', 'latex');
ylabel('Erro [g]')
xlabel('Tempo [s]')

% Plot M
figure('Name','M')
plot(t_vec,y_traj(2,:),'Color','k')
hold
plot(t_vec,ympc_traj(2,:),'Color','r')
ylim([1 4])
hh = title('N\''umero de Mach ($M$)');
set(hh, 'Interpreter', 'latex');
ylabel('Número de Mach')
xlabel('time [s]')

% Plot x(7) (delta_dot)
figure('Name','delta_dot')
plot(t_vec,x_traj(7,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(7,:)/d2r,'Color','k')
yline([50 -50],'--r',{'Max','Min'},'LineWidth',2)
ylim([-55 55])
hh = title('Taxa de deflex\~ao da aleta ($\dot{\delta}$)');
set(hh, 'Interpreter', 'latex');
ylabel('Taxa de deflexão da aleta [°/s]')
xlabel('Tempo [s]')

% Plot u(2) (delta_c)
figure('Name','delta_c')
stairs(t_vec,u_traj(2,:)/d2r,'Color','k')
yline([45 -45],'--r',{'Max','Min'},'LineWidth',2)
ylim([-50 50])
hh = title('Comando de deflex\~ao da aleta ($\delta^{com}$)');
set(hh, 'Interpreter', 'latex');
ylabel('Comando de deflexão da aleta [°]')
xlabel('Tempo [s]')

% Plot proc_time
figure('Name','proc_time')
plot(t_vec(1:end-1),proc_time*1e3,'Color','k')
yline(2e1,'--r',{'Max'},'LineWidth',2)
ylim([0 2.2e1])
hh = title('Tempo de processamento');
set(hh, 'Interpreter', 'latex');
ylabel('Tempo de processamento por amostra [ms]')
xlabel('Tempo [s]')

% Plot proc_time histogram
figure('Name','proc_time histogram')
histogram(proc_time*1e3)
hh = title('Histograma do tempo de processamento');
set(hh, 'Interpreter', 'latex');
ylabel('Frequência [# de amostras]')
xlabel('Tempo de processaemnto por amostra [ms]')

% Plot x(1)
figure('Name','alpha')
plot(t_vec,x_traj(1,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(1,:)/d2r,'Color','r')
yline([-20 20],'--r',{'Min','Max'},'LineWidth',2)
ylim([-25 25])
hh = title('\^Angulo de Ataque ($\alpha$)');
set(hh, 'Interpreter', 'latex');
ylabel('\alpha [°]')
xlabel('Tempo [s]')

% Plot x(2)
figure('Name','M')
plot(t_vec,x_traj(2,:),'Color','k')
hold
plot(t_vec,xmpc_traj(2,:),'Color','r')
ylabel('M')
xlabel('time [s]')

% Plot x(3)
figure('Name','q')
plot(t_vec,x_traj(3,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(3,:)/d2r,'Color','r')
ylabel('q [°/s]')
xlabel('time [s]')

% Plot x(4)
figure('Name','h')
plot(t_vec,x_traj(4,:),'Color','k')
hold
plot(t_vec,xmpc_traj(4,:),'Color','r')
ylabel('h [m]')
xlabel('time [s]')

% Plot x(5)
figure('Name','theta')
plot(t_vec,x_traj(5,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(5,:)/d2r,'Color','r')
ylabel('theta [°]')
xlabel('time [s]')

% Plot x(6)
figure('Name','delta')
plot(t_vec,x_traj(6,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(6,:)/d2r,'Color','r')
yline([45 -45],'--r',{'Max','Min'},'LineWidth',2)
ylim([-50 50])
hh = title('\^Angulo de deflex\~ao da aleta ($\delta$)');
set(hh, 'Interpreter', 'latex');
ylabel('\delta [°]')
xlabel('Tempo [s]')

% Plot x(7)
figure('Name','delta_dot')
plot(t_vec,x_traj(7,:)/d2r,'Color','k')
hold
plot(t_vec,xmpc_traj(7,:)/d2r,'Color','r')
ylabel('delta_dot [°/s]')
xlabel('time [s]')

function [cost, uk] = mpc_solve(mpc, x0, x0_sym, p0, p0_sym, yref, yref_sym, U_sym)
    % Set initial state and reference
    mpc.set_value(x0_sym, x0);
    mpc.set_value(p0_sym, p0);
    mpc.set_value(yref_sym, yref);

    % Solve the optimization problem
    sol = mpc.solve_limited();
    % assert(sol.stats.success == 1, 'Error calculating solution');
    
    uk = sol.value(U_sym(:,1));

    % Objective cost value
    cost = mpc.value(mpc.f);
    
    % Use the current solution to speed up the next optimization
    mpc.set_initial(sol.value_variables());
    % mpc.set_initial(mpc.lam_g, sol.value(mpc.lam_g));
end

function [cost, pk] = mhe_solve(mhe, uk, uk_sym, yk, yk_sym, Pe_sym)
    % Set parameters
    mhe.set_value(uk_sym, uk);
    mhe.set_value(yk_sym, yk);

    % Solve the optimization problem
    sol = mhe.solve_limited();
    % assert(sol.stats.success == 1, 'Error calculating solution');
    
    pk = sol.value(Pe_sym(:,1));

    % Objective cost value
    cost = mhe.value(mhe.f);

    % Set last solution as initial guess
    mhe.set_initial(Pe_sym,pk);
end

function V_N_plot(x_0, x_r, u_r, p, y_r, N, P, Q, R, model_full)

% Grid for u0 and u1
u0 = linspace(-0.7854, 0.7854, 25);
u1 = linspace(-0.7854, 0.7854, 25);
[U0, U1] = meshgrid(u0, u1);

% Initialize x, u and y
x = zeros(size(x_r, 1), N, numel(u1), numel(u0));
u = zeros(size(u_r, 1), N-1, numel(u1), numel(u0));
y = zeros(numel(u1), numel(u0));

% Compute x_k for each pair [u0, u1]
for i = 1:numel(u0)
    for j = 1:numel(u1)
        % Initial state
        x(:,1,j,i) = x_0;

        u(:,1,j,i) = u0(i);
        u(:,2,j,i) = u1(j);

        for k = 1:N-1
            x(:,k+1,j,i) = full(model_full.f_x_nxt(x(:,k,j,i),[0; u(:,k,j,i)],p));
            % x(:,k+1,j,i) = full(model_full.f_x_nxt_lin(x_0, (x(:,k,j,i) - x_0),[0; u(:,k,j,i)],p));
            if k > 2 && k < N
                u(:,k,j,i) = u(:,2,j,i);
            end
        end

        y_N = full(model_full.f_y(x(:,k+1,j,i),p));
        y(j,i) = y_N(4);
    end
end

% Cost function V_N
V_N = @(x, u, x_r, u_r, y_r, y_N, P, Q, R) sum((x - x_r)' * Q * (x - x_r) + sum((u - u_r)' * R * (u - u_r))) + (y_N - y_r)' * P * (y_N - y_r);

% Compute V_N for each point in grid
V_values = zeros(size(U0));
for i = 1:numel(u0)
    for j = 1:numel(u1)
        V_values(j, i) = V_N(x(:,end-1,j,i), u(:,:,j,i), x_r, u_r, y_r, y(j,i), P, Q, R);
    end
end

% Plot V_N
figure('Name','V_N for x0')
surf(U0, U1, V_values);
xlabel('u_0');
ylabel('u_1');
zlabel('V_N');
title('Gráfico 3D de V_N em função de u_0 e u_1');

end