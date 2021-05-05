function [delta, Fx ] = group7_controller( s, e, dpsi, Ux, Uy, r, control_mode, path)
%ME227 Controller:
% Spring 2021
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda, Alaisha
% Alexander, Will Harvey, Lucio Mondavi, John Talbot, Trey Weber
% 
persistent e_history;
if isempty(e_history)
    e_history = e;
end
e_history = [e_history, e];
%--------------------------------------------------------------------------
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

%% Drag and Friction Constants (Satyan)

f_rr = 0.015; % rolling friction given in project file
Cd_A = 0.594; %product of drag coefficient and frontal area
rho=1.225; %air density at room temperature

%--------------------------------------------------------------------------
%% Find Path Dependent Parameters (Satyan)
%--------------------------------------------------------------------------
 kappa = interp1(path.s_m, path.k_1pm, s); %typical value
 %kappa=0; % but assuming kappa = 0 for first part i.e. straight road
 UxDes = interp1(path.s_m,path.UxDes,s); %desired speed at given 's' value per path
 axDes = interp1(path.s_m,path.axDes,s); %desired acceleration at given 's'

%--------------------------------------------------------------------------
%% Control Parameters (Satyan)
%--------------------------------------------------------------------------
gains.k_la = 3500; %arbitrarily chosen, NEEDS ADJUSTMENT
gains.x_la = 8; %arbitrarily chosen, NEEDS ADJUSTMENT
gains.k_lo = m*0.3*g/1; %assumed placehoder from HW 4. NEEDS ADJUSTMENT

Kp = 1;
Kd = 0.5;
Ki = 1;

%--------------------------------------------------------------------------
%% Lateral Control Law (Satyan)
%--------------------------------------------------------------------------
%Use the Lateral Control Law to Calculate Delta

if control_mode == 1 %Lookahead Controler
    
    dpsi_ss=kappa*((m*a*Ux^2)/(L*r_tire.Ca_lin)-b); %steady state dpsi (heading error), HW4
    K_grad=((Wf/f_tire.Ca_lin)-(Wr/r_tire.Ca_lin))/g; %understeer gradient, HW 2
    delta_ff=(gains.k_la*gains.x_la*dpsi_ss/f_tire.Ca_lin) + kappa*(L + (K_grad*Ux^2)); %delta feedforward term, HW 4
    delta=(-gains.k_la*(e+(gains.x_la*dpsi))/f_tire.Ca_lin)+delta_ff; %feedforward + feedback delta (steer angle), HW4
    
else %Your second controller

    % not started, done yet
    dt = 0.001; %s, taken from hard simulation code
    e_dot = Uy*cos(dpsi) + Ux*sin(dpsi);
    delta = -(Kp*e + Kd*e_dot + Ki*trapz(dt, e_history));
    
    %if the delta value goes over a certain value cap it at that value (for
    %both negative and positive) Anti-windup
    value = 10; %not an actual value just a placeholder. Value should be in radians
    if (delta > value)
        delta = value;
    elseif (delta < -value)
        delta = -value;
    end
end

%--------------------------------------------------------------------------
%% Longitudinal Control Law
%--------------------------------------------------------------------------
%Use the Longitudinal Control Law to Calcuate Fx

F_rr = f_rr*m*g;
F_d = 0.5*Cd_A*rho*Ux^2;
F_des=m*axDes;
Fx=F_des + F_rr + F_d + (m*gains.k_lo*(UxDes-Ux));





end
