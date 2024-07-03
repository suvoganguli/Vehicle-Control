
clear

%% Car Parameters

m = 1724; % kg
Iz = 3400; % kg/m^2
a = 1.35; % m
b = 1.15; % m
Cf = 12*1e4; % N/rad
Cr = 175*1e3; % N/rad
mu = 0.5225;

Vx = 10/3.6; % mph to mps

g = 9.81; % mps2

%% Open Loop

% x = [ydot r] 
% u = delta

A = [-(Cf+Cr)/(m*Vx)  -(a*Cf-b*Cr)/(m*Vx)-Vx; 
    -(a*Cf-b*Cr)/(Iz*Vx) -(a^2*Cf+b^2*Cr)/(Iz*Vx)];

B = [Cf/m; a*Cf/Iz];

w = 10;

% setting for open loop linmod
intgA = [];
intgB = [];
intgC = [];
intgD = eye(2);

C = [0 0 0];
CA = [0 0 0];
invCB = 0;
K_ay = 0;
K_v = 0;
loop = 0;
rho = 0;

% warning off
% model = 'sim_linear_control_y';
% [Aol,Bol,Col,Dol] = linmod(model);
% warning on

% after adding filter
Aol = [A(1,1)      A(1,2)       0;
       A(2,1)      A(2,2)       0;
       w*A(1,1)  w*(A(1,2)+Vx) -w]; 

Bol = [B; B(1)];

Col = [eye(3); Aol(3,:)];

Dol = [zeros(3,1); Bol(3)];


% open loop poles
disp('Open Loop Poles');
rifd(eig(Aol))
disp(' ');

olp = ss(Aol,Bol,Col,Dol);

t = (0:0.01:5)';
nt = length(t);
delta = zeros(nt,1);
delta(101:300) = 10*pi/180;
out = lsim(olp,delta,t);

figure(1);
subplot(211);
plot(t,out(:,[3 4])); grid on;
ylabel('yddot, yddotf [m/s]');
subplot(212);
plot(t,out(:,2)*180/pi); grid on;
ylabel('r [deg/s]');
xlabel('t [s]');

%% Control Law

C = Col(3,:);
CA = C*Aol;
invCB = pinv(C*Bol);

K_ay = 4;
K_v = 1;

loop = 1;

warning off
[Acl,Bcl,Ccl,Dcl] = linmod(model);
warning on

disp('Closed Loop Poles');
rifd(eig(Acl))

%% simulation

intgA = zeros(2,2);
intgB = eye(2);
intgC = eye(2);
intgD = zeros(2,2);

R = 20;
rho = 1/R;

sim('sim_linear_control_y');

