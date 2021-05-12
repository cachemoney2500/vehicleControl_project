function [delta, Fx ] = group7_controller( s, e, dpsi, Ux, Uy, r, control_mode, path)
%ME227 Controller:
% Spring 2021
% Prof. Chris Gerdes & CAs Nathan Spielberg, John Alsterda, Alaisha
% Alexander, Will Harvey, Lucio Mondavi, John Talbot, Trey Weber


%NOTE (Satyan) - anything marked with a '??' needs better conceptual reasoning and
%final adjustment
dt = 0.001;
persistent e_history;
if isempty(e_history)
    e_history = 0;
end

if Ux > 0
    e_history = e_history + dt*e;
end


%e_history = [e_history, e];

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
gains.k_la = 10000; %arbitrarily chosen, ??
gains.x_la = 10; %arbitrarily chosen, ??
gains.k_lo = m*2; %m*.04*g; %arbitrarily chosen, ??

Kp = 1.34; % arbitrarily chosen, ??
Kd = 0.31;  % arbitrarily chosen, ??
Ki = 2.76;  % arbitrarily chosen, ??

%--------------------------------------------------------------------------
%% Lateral Control Law (Satyan)
%--------------------------------------------------------------------------
%Use the Lateral Control Law to Calculate Delta

if control_mode == 1 %Lookahead Controler
    %started with linear stiffnesses, moved to fiala stiffnesses
    
    dpsi_ss=kappa*((m*a*Ux^2)/(L*r_tire.Ca_lin)-b); %nonlinear steady state dpsi (heading error), HW4
    K_grad=((Wf/f_tire.Ca_lin)-(Wr/r_tire.Ca_lin))/g; %understeer gradient, HW 2
    delta_ff=(gains.k_la*gains.x_la*dpsi_ss/f_tire.Ca_lin) + kappa*(L + (K_grad*Ux^2)); %nonlinear delta feedforward term, HW 4
    delta=(-gains.k_la*(e+(gains.x_la*dpsi))/f_tire.Ca_lin)+delta_ff; %feedforward + feedback delta (steer angle), HW4
    

else %Your second controller

    % done yet
    e_dot = Uy*cos(dpsi) + Ux*sin(dpsi);
    %delta = -(Kp*e + Kd*e_dot + Ki*trapz(dt, e_history));
    delta = -(Kp*e + Kd*e_dot + Ki*e_history);
    
    %if the delta value goes over a certain value cap it at that value (for
    %both negative and positive) Anti-windup
    value = deg2rad(20);
    if (delta > value)
        delta = value;
    elseif (delta < -value)
        delta = -value;
    end
end

% OBSERVATIONS FROM LATERAL ERROR CODE
% 1. Linear stiffnesses give better error, tracking response than fiala ??
% 2. Higher cornering stiffness = higher lateral forces = higher lateral
% accel = lower understeer gradient = lower delta input = more worse
% tracking.
% 3. Changing x_la changed (UxDes - Ux). Rationale unclear ??
% 4. Can we ignore first turn Ux and Ax/Ay/ATotal?
%--------------------------------------------------------------------------
%% Longitudinal Control Law
%--------------------------------------------------------------------------
%Use the Longitudinal Control Law to Calcuate Fx

F_rr = f_rr*m*g;
F_d = 0.5*Cd_A*rho*Ux^2;
F_des=m*axDes;
Fx=F_des + F_rr + F_d + gains.k_lo*(UxDes-Ux); %% ??, not following lambda term in notes

% LONGITUDINAL CONTROLLER THOUGHTS AND CONCEPTS CHECK
% If density increases, F_d increases, Fx increases, gain same OK
% If f_rr increases (low tire pressure), F_rr up, Fx up, gain same OK
% If UxDes < Ux, feedback term -ve, Fx lowers, lowers Ux
% If UxDes > Ux, feedback term +ve, Fx rises, rises Ux
% ?? Lambda gain term is unclear, needs thought ??

% PARAMETERS TO LOOK AT FOR CHOOSING LONGITUDINAL GAIN
% 1. Acceleration: want to keep < 0.3 g. If Ax_accel goes up, lower gain
% 2. Deceleration: want to keep < -0.4g. If Ax_decel goes up, lower gain
% 3. Extract difference between 0 gain and some gain. How's performance?
% 4. Fx typically 500-600N without gain term. delta_Ux = 1.2 m/s
% 5. Add 500 N of gain assist so k_lo = 500/ 1.2 = 416 N/m = m*0.023*g
% 6. In braking, without gain term, UxDes > ux. Adding gain will correct Ux
% 7. m*0.023*g gets Speed Error < 0.75 m/s and keeps peak Ax < 1.68g





end
