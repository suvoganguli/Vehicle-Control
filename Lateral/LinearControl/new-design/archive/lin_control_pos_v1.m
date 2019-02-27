
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

% after adding filter
% x = [ydot r ydotf y psi]
Aol = [A(1,1)      A(1,2)       0   0   0;
       A(2,1)      A(2,2)       0   0   0;
       w*A(1,1)  w*(A(1,2)+Vx) -w   0   0;
       1           0            0   0   0;
       0           1            0   0   0]; 

Bol = [B; w*B(1); 0; 0];

% x ydot
Col = [eye(5); Aol(1,:)];

Dol = [zeros(5,1); Bol(1,:)];

% open loop poles
disp('Open Loop Poles');
rifd(eig(Aol))
disp(' ');

%% Control Law

C = Col(3,:);  % ydotf
CA = C*Aol;
invCB = pinv(C*Bol);

K_ay = 8;
K_Vy = 4;
K_chi = 2;

%% simulation

R = 20;
rho = 1/R;
dchic = 0.1*pi/180;
chic = (0 : dchic : 90*pi/180)';
chic = [chic; 90*pi/180*ones(5000,1)];
nt = length(chic);
Vxc = Vx*ones(nt,1);
Ndotc = Vxc.*cos(chic);
Edotc = Vxc.*sin(chic);

dt = 0.01;
t = (0 : dt : (nt-1)*dt)';

Nc = cumsum(Ndotc)*dt;
Ec = cumsum(Edotc)*dt;

sim_Nc = [t Nc];
sim_Ec = [t Ec];

model = 'sim_linear_control_pos';

warning off
[Acl,Bcl,Ccl,Dcl] = linmod(model);
warning on

disp('Closed Loop Poles');
rifd(eig(Acl))

sim(model);

%% N-E

N = cumsum(Vx*cos(chi))*dt;
E = cumsum(Vx*sin(chi))*dt;

%% plots

figure(1);
subplot(211);
plot(t, x(:,1)/g); grid on
ylabel('ydot [g]');
subplot(212);
plot(t, x(:,2)*180/pi); grid on
ylabel('r [deg/s]');
xlabel('t [sec]');

figure(2);
subplot(211);
plot(t, x(:,3)/g); grid on
ylabel('ayf [g]');
subplot(212);
plot(t, x(:,5)*180/pi); grid on
ylabel('psi [deg/s]');
xlabel('t [sec]');

figure(3);
subplot(211);
plot(t, chi*180/pi, t, chic*180/pi); grid on
ylabel('chi, chic [deg]');
subplot(212);
plot(t,beta*180/pi); grid on
ylabel('beta [deg]');
xlabel('t [sec]');

figure(4);
subplot(211);
plot(t, N, t, Nc); grid on
ylabel('N, Nc [m]');
subplot(212);
plot(t, E, t, Ec); grid on
ylabel('E, Ec [m]');
xlabel('t [sec]');






