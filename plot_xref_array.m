clear
clc
close all

% 3D-Plot

load('xref.mat')
load('missile_data.mat')

az_vec = 0:10:350; % n=36
M_vec = 1.5:0.1:4.5; % n=31

% 3D-Plot of each variable as a function of (az,M)
x1ref_mat = reshape(xref_mat(1,:,:), length(az_vec), length(M_vec));
x2ref_mat = reshape(xref_mat(2,:,:), length(az_vec), length(M_vec));
x3ref_mat = reshape(xref_mat(3,:,:), length(az_vec), length(M_vec));
x4ref_mat = reshape(xref_mat(4,:,:), length(az_vec), length(M_vec));

figure('Name','alpha_ref')
surf(M_vec,-az_vec,-x1ref_mat/d2r)
hold
surf(M_vec,az_vec,x1ref_mat/d2r)
colorbar
xlabel('M_0')
ylabel('a_Z')
zlabel('alpha [°]')
hh = title('\^Angulo de ataque ($\alpha_r$)');
set(hh, 'Interpreter', 'latex');
xlim([1.5 4.5])
xticks(1.5:0.5:4.5)
ylim([-350 350])
yticks([-350 -300 -200 -100 0 100 200 300 350]);

figure('Name','M_ref')
surf(M_vec,-az_vec,x2ref_mat)
hold
surf(M_vec,az_vec,x2ref_mat)
colorbar
xlabel('M_0')
ylabel('a_Z')
zlabel('M')
hh = title('N\''umero de Mach ($M_r$)');
set(hh, 'Interpreter', 'latex');
xlim([1.5 4.5])
xticks(1.5:0.5:4.5)
ylim([-350 350])
yticks([-350 -300 -200 -100 0 100 200 300 350]);

figure('Name','q_ref')
surf(M_vec,-az_vec,-x3ref_mat/d2r)
hold
surf(M_vec,az_vec,x3ref_mat/d2r)
colorbar
xlabel('M_0')
ylabel('a_Z')
zlabel('q [°/s]')
hh = title('Velocidade angular de arfagem ($q_r$)');
set(hh, 'Interpreter', 'latex');
xlim([1.5 4.5])
xticks(1.5:0.5:4.5)
ylim([-350 350])
yticks([-350 -300 -200 -100 0 100 200 300 350]);

figure('Name','delta_ref')
surf(M_vec,-az_vec,-x4ref_mat/d2r)
hold
surf(M_vec,az_vec,x4ref_mat/d2r)
colorbar
xlabel('M_0')
ylabel('a_Z')
zlabel('delta [°]')
hh = title('\^Angulo de deflex\~ao do atuador ($\delta_r$)');
set(hh, 'Interpreter', 'latex');
xlim([1.5 4.5])
xticks(1.5:0.5:4.5)
ylim([-350 350])
yticks([-350 -300 -200 -100 0 100 200 300 350]);

% Create a grid
[i, j] = meshgrid(1:size(x1ref_mat,2), 1:size(x1ref_mat,1));

% Plot the surface using the matrices as coordinates
figure('Name','Superfície de Equilíbrio');
surf(x1ref_mat/d2r, x3ref_mat/d2r, x4ref_mat/d2r);
colorbar
xlabel('\alpha_r [°]');
ylabel('q_r [°/s]');
zlabel('\delta_r [°]');
title('Superfície de Equilíbrio em R^3 [\alpha_r,q_r,\delta_r]^T');