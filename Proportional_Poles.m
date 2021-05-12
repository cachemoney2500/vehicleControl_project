%--------------------------------------------------------------------------
%% HEADER
%--------------------------------------------------------------------------
% ME227 Spr 2021
% Homework 4 - Question 1.E

%--------------------------------------------------------------------------
%% CONSTANTS AND PARAMS
%--------------------------------------------------------------------------
% Load Niki params
setup_niki;

% Gains and conditions
Kp_ = 1.1;   % [rad/m]  
Ux = 10;       % [m/s]
lenKla = length(Kp_);

% Allocate space for poles (We know there are 4)
poles_ = zeros(5,lenKla);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for idx = 1:lenKla
    % Select speed
    Kp = Kp_(idx);
    
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
    eM = -Kp*f_tire.Ca_lin/veh.m;
    fM = -(f_tire.Ca_lin+r_tire.Ca_lin)/(veh.m*Ux);
    gM = (f_tire.Ca_lin+r_tire.Ca_lin)/veh.m;
    hM = (-veh.a*f_tire.Ca_lin+veh.b*r_tire.Ca_lin)/(veh.m*Ux);
    iM = 0;
    jM = 0;
    kM = 0;
    lM = 1;
    mM = -Kp*veh.a*f_tire.Ca_lin/veh.Iz;
    nM = (veh.b*r_tire.Ca_lin-veh.a*f_tire.Ca_lin)/(veh.Iz*Ux);
    oM = (-veh.b*r_tire.Ca_lin+veh.a*f_tire.Ca_lin)/veh.Iz;
    pM = -(veh.b^2*r_tire.Ca_lin+veh.a^2*f_tire.Ca_lin)/(veh.Iz*Ux);

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
    newcol = zeros(4,1);
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
cbar = colorbar('Ticks', Kp_);
caxis([Kp_(1) Kp_(end)])
cbar.Label.String = 'K_{p} [N/m]';
