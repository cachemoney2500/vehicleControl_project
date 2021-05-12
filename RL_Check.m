clc
clear all
close all
%% Constants
%--------------------------------------------------------------------------
g = 9.81;                       % [m/s^2]  gravity

%--------------------------------------------------------------------------
%% Vehicle Parameters
%--------------------------------------------------------------------------
m  = 1776;                  % [kg]     mass with 2 occupants
Iz = 2763.49;               % [kg-m^2] rotational inertia
a  = 1.264;                 % [m]      distance from CoM to front axle
b  = 1.367;                 % [m]      distance from C0M to rear axle
L  = a + b;         % [m]      wheelbase
Wf = m*g*(b/L); % [N]      static front axle weight
Wr = m*g*(a/L); % [N]      static rear axle weight

%--------------------------------------------------------------------------
%% Tire Parameters
%--------------------------------------------------------------------------
% Front tires
f_tire.Ca_lin = 80000;          % [N/rad]  linear model cornering stiffness
f_tire.Cy     = 110000;         % [N/rad]  fiala model cornering stiffness
f_tire.mu_s   = 0.90;           %          sliding friction coefficient
f_tire.mu     = 0.90;           %          peak friction coefficient

% Rear tires
r_tire.Ca_lin = 120000;
r_tire.Cy     = 180000;
r_tire.mu_s   = 0.94;
r_tire.mu     = 0.94;


%% Transfer function and root locus
s = tf('s');
Ux = 10;
tau = 1*10^-10;

num1 = f_tire.Ca_lin*Iz/m;
num2 = b*L*f_tire.Ca_lin*r_tire.Ca_lin/(m*Ux);
num3 = L*f_tire.Ca_lin*r_tire.Ca_lin/m;
num = (num1*s^2 + num2*s + num3);

den1 = Iz;
den2 = Iz*(f_tire.Ca_lin + r_tire.Ca_lin)/(m*Ux) + (a^2*f_tire.Ca_lin + b^2*r_tire.Ca_lin)/Ux;
den3 = f_tire.Ca_lin*r_tire.Ca_lin*L^2/(m*Ux^2) + b*r_tire.Ca_lin - a*f_tire.Ca_lin;
den = s^2*(den1*s^2 + den2*s + den3);

TF = num/den;

rlocus(TF)