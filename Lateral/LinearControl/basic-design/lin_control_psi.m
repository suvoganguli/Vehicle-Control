m = 1724; % kg
Iz = 3400; % kg/m^2
a = 1.35; % m
b = 1.15; % m
Cf = 12*1e4; % N/rad
Cr = 175*1e3; % N/rad
mu = 0.5225;

Vx = 30/3.6; % mps

% x = [vy r] 
% u = delta

A = [-(Cf+Cr)/(m*Vx)  -(a*Cf-b*Cr)/(m*Vx)-Vx; 
    -(a*Cf-b*Cr)/(Iz*Vx) -(a^2*Cf+b^2*Cr)/(Iz*Vx)];

B = [Cf/m; a*Cf/Iz];

C = [0 1];
CA = C*A;
invCB = pinv(C*B);

Kb_r = 4;
Kb_psi = 1;

%% simulation

sim('sim_linear_control_psi');

