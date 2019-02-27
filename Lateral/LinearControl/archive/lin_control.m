m = 1724; % kg
Iz = 3400; % kg/m^2
a = 1.35; % m
b = 1.15; % m
Cf = 12*1e4; % N/rad
Cr = 175*1e3; % N/rad
mu = 0.5225;

Vx = 30/3.6; % mps

% x = [vy Chidot], u = delta

A = [-(Cf+Cr)/m  -(a*Cf-b*Cr)/(m*Vx)-Vx; 
    -(a*Cf-b*Cr)/Iz -(a^2*Cf-b^2*Cr)/(Iz*Vx)];

B = [Cf/m; a*Cf/Iz];

% sys = ss(A,B,eye(2),zeros(2,1));
% 
% t = (0:0.001:1)';
% n = length(t);
% y = lsim(sys,30*pi/180*ones(n,1),t);
% 
% figure(1);
% plot(t,y(:,1),t,y(:,2)*180/pi)

C = [1 0;
     0 1];
CA = C*A;
invCB = pinv(C*B);

Kb = 4;
fi = 0.25;
fc = 0.5;

%% simulation

sim('sim_linear_control');

