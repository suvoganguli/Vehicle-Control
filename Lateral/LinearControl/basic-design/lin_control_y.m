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

C = [1 0];
CA = C*A;
invCB = pinv(C*B);

Kb_Vy = 8;
Kb_y = 2;
fi = 0.25;
fc = 0.5;

%% simulation

sim('sim_linear_control_y');

