%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2021
% Homework 4 - Question 1.E
clc
clear all
close all
%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Gains and conditions
Kp_ = 1.2;   % [N/m]
Ki_ = 2.4:0.01:2.5;
Kd_ = 0.31;           % [m]    
Ux = 10;       % [m/s]
lenKla = length(Ki_);

% Allocate space for poles (We know there are 4)
poles_ = zeros(5,lenKla);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for idx = 1:lenKla
    % Select speed
    Ki = Ki_(idx);
    
    %{
    To make the state matrix easier to input, create each term separately
    here according to this template - we'll complile these into the matrix
    at the end. We recommend you keep this general and let MATLAB fill in
    each of the values as you set them up above. Then you can copy and
    paste this section into later problems.
    
        A = [aM,  bM,  cM,  dM]
            [eM,  fM,  gM,  hM]
            [iM,  jM,  kM,  lM]
            [mM,  nM,  oM,  pM]
    %}
    
    aM = 0;
    bM = 1;
    cM = 0;
    dM = 0;
    eM = -Kp_*f_tire.Ca_lin/veh.m;
    fM = -(f_tire.Ca_lin+r_tire.Ca_lin)/(veh.m*Ux)-Kd_*f_tire.Ca_lin/veh.m;
    gM = (f_tire.Ca_lin+r_tire.Ca_lin)/veh.m;
    hM = (-veh.a*f_tire.Ca_lin+veh.b*r_tire.Ca_lin)/(veh.m*Ux);
    iM = 0;
    jM = 0;
    kM = 0;
    lM = 1;
    mM = -Kp_*veh.a*f_tire.Ca_lin/veh.Iz;
    nM = (veh.b*r_tire.Ca_lin-veh.a*f_tire.Ca_lin)/(veh.Iz*Ux)-Kd_*veh.a*f_tire.Ca_lin/veh.Iz;
    oM = (-veh.b*r_tire.Ca_lin+veh.a*f_tire.Ca_lin)/veh.Iz;
    pM = -(veh.b^2*r_tire.Ca_lin+veh.a^2*f_tire.Ca_lin)/(veh.Iz*Ux);

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
     
    newcol = [0;
               -Ki*f_tire.Ca_lin/veh.m;
               0
               -Ki*veh.a*f_tire.Ca_lin/veh.Iz];
    A = [newcol A];
    
    newrow = [0 1 0 0 0];
    A = [newrow; A];
    
   % Calculate pole positions
   poles_(:,idx) = eig(A);
end

%--------------------------------------------------------------------------
%% PLOT RESULTS
%--------------------------------------------------------------------------
figure
cmap = colormap(winter(lenKla));
for idx = 1:lenKla
    plot(real(poles_(:,idx)), imag(poles_(:,idx)), 'x', 'Color', cmap(idx,:))
    hold on
end
xline(0,'--');
grid on
xlabel('Real Axis')
ylabel('Imaginary Axis')
cbar = colorbar('Ticks', Ki_);
caxis([Ki_(1) Ki_(end)])
cbar.Label.String = 'K_{p} [N/m]';
