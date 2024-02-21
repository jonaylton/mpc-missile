%% Missile Data
%==================================================================
% Useful Constants
%==================================================================

d2r     = pi/180;                 % Conversion Deg to Rad
g       = 9.80665;                % Gravity [m/s^2]
m2ft    = 3.28084;                % metre to feet
Kg2slug = 0.0685218;              % Kg to slug

%==================================================================
% Atmospheric Constants
%==================================================================

T0      = 288.15;                 % Temp. at Sea Level [K]
rho0    = 1.225;                  % Density [Kg/m^3]
L       = 0.0065;                 % Lapse Rate [K/m]
R       = 287.0531;               % Gas Constant J/Kg/K
gam     = 1.403;                  % Ratio of Specific Heats
P0      = 101325.0;               % Pressure at Sea Level [N/m^2]
h_trop  = 11000.0;                % Height of Troposphere [m]
Vs0     = 340.2787;               % Speed of Sound at Sea Level [m/s]

%==================================================================
% Missile Configuration
%==================================================================
S_ref   = 0.44/m2ft^2;            % Reference area [m^2]
d_ref   = 0.75/m2ft;              % Reference length [m]
Iyy     = 182.5/(Kg2slug*m2ft^2); % Inertia
mass    = 13.98/Kg2slug;          % Mass [Kg]
Thrust  = 10e3;                   % Thrust [N]

%==================================================================
% Missile Aerodynamics
%==================================================================

% Axial Force Coefficient
aa = -0.3;

% Normal Force Coefficient
an    =  0.000103/d2r^3;      % [rad^-3]
bn    = -0.009450/d2r^2;      % [rad^-2]
cn    = -0.169600/d2r;	      % [rad^-1]
dn    = -0.034000/d2r;        % [rad^-1]

% Moment Coefficient
am       =  0.000215/d2r^3;   % [rad^-3]
bm       = -0.019500/d2r^2;   % [rad^-2]
cm       =  0.051000/d2r;     % [rad^-1]
dm       = -0.206000/d2r;     % [rad^-1]
em       = -1.719;            % [s/rad]

%==================================================================
% Define Initial Conditions
%==================================================================
x_ini      = 0;		        % Initial downrange position [m]
h_ini      = 20000/m2ft;    % Initial altitude [m]
Vs_ini     = sqrt((T0-L*h_ini)*gam*R);  % Speed of sound eq. for 0m <= h <= 11000m
M_ini      = 4;             % Initial Mach Number
v_ini      = M_ini*Vs_ini;	% Initial velocity [m/s]
alpha_ini  = 0*d2r;		    % Initial incidence [rad]
theta_ini  = 0*d2r;		    % Initial Body Attitude [rad]
q_ini      = 0*d2r;		    % Initial pitch rate [rad/sec]

%==================================================================
% Missile Actuators
%==================================================================
wn_fin      =  150.0;       % Actuator Bandwidth [rad/sec]
z_fin 	    =  0.7;         % Actuator Damping
fin_act_0   =  0.0;         % Initial Fin Angle [rad]
fin_max     =  45.0*d2r;    % Max Fin Deflection [rad] 
fin_min	    = -45.0*d2r;    % Min Fin Deflection [rad]
fin_maxrate =  50*d2r;      % Max Fin Rate [rad/sec]

save('missile_data.mat')