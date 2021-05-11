setup_niki;

% Gains and conditions
K_la_ = 1000:1000:10000;   % [N/m]
x_la = 18;     % [m]% keep k_la varying, adjust x_la in 3 m incr (3,6,9,12,15,18)
% x_la 3 m to 9 m is unstable, 12 and 15 are sweet spot, 18 is oscillatory
Ux = 6;       % [m/s]
lenKla = length(K_la_);

poles_ = zeros(4,lenKla);

%--------------------------------------------------------------------------
%% CREATE SYSTEM MATRIX
%--------------------------------------------------------------------------
for idx = 1:lenKla
    % Select speed
    K_la = K_la_(idx);
    
    aM = 0;
    bM = 1;
    cM = 0;
    dM = 0;
    eM = -K_la_(idx)/veh.m;
    fM = -((f_tire.Ca_lin+r_tire.Ca_lin)/(veh.m*Ux));
    gM = ((f_tire.Ca_lin+r_tire.Ca_lin)/veh.m) - (K_la_(idx)*x_la/veh.m);
    hM = (-veh.a*f_tire.Ca_lin + (veh.b*r_tire.Ca_lin))/(veh.m*Ux);
    iM = 0;
    jM = 0;
    kM = 0;
    lM = 1;
    mM = -K_la_(idx)*veh.a/veh.Iz;
    nM = ((veh.b*r_tire.Ca_lin)-(veh.a*f_tire.Ca_lin))/(veh.Iz*Ux);
    oM = (veh.a*f_tire.Ca_lin - (veh.b*r_tire.Ca_lin))/(veh.Iz) - (K_la_(idx)*veh.a*x_la)/veh.Iz;
    pM = -(veh.a^2*f_tire.Ca_lin + (veh.b^2*r_tire.Ca_lin))/(veh.Iz*Ux);

    A = [[aM,  bM,  cM,  dM];
         [eM,  fM,  gM,  hM];
         [iM,  jM,  kM,  lM];
         [mM,  nM,  oM,  pM]];
    
   % Calculate pole positions
   poles_(:,idx) = eig(A);
end

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
cbar = colorbar('Ticks', K_la_);
caxis([K_la_(1) K_la_(end)])
cbar.Label.String = 'K_{la} [N/m]';