close all
clear
clc

% Constants
gamma = 1.403;
T0 = 288.15;
g = 9.80665;
rho_0 = 1.225;
L = 0.0065;
R = 287.0531;
h = 6096; % Altitude, m

% Function for calculating speed of sound Vs
Vs = @(h) sqrt(gamma * R * (T0 - L * h));

% Functions Q1 e Q2
Q1 = @(M, h) ((rho_0 * ((1 - (L/T0)*h) .^ ((g/(R*L)) - 1))) .* R .* (T0 - L * h)) .* ...
    (((1 + ((gamma - 1)/2).*(M.^2)) .^ (gamma/(gamma - 1))) - 1);

Q2 = @(M, h) 0.5*(rho_0 * ((1 - (L/T0)*h) .^ ((g/(R*L)) - 1))) .* (Vs(h).^2) .* (M.^2) .* ...
    (1 + M.^2/4 + M.^4/40 + M.^6/2100);

% Mach values
M_values = linspace(2, 4, 100);

% Calculating Qexact and Qaprox
Q1_values = Q1(M_values, h);
Q2_values = Q2(M_values, h);

% Plot
figure;
plot(M_values, Q1_values, 'b-', 'LineWidth', 1, 'DisplayName', 'Q exato (h=6096m)');
hold on;
plot(M_values, Q2_values, 'r--', 'LineWidth', 2, 'DisplayName', 'Q aproximado (h=6096m)');
title(sprintf('Comparação entre Q exato e Q aproximado\npara h = 6096m'));
xlabel('Número de Mach (M)');
ylabel('Pressão Dinâmica (Q) [N/m²]');
legend('show');
grid on;