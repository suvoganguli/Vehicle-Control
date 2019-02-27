
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

% x = [ydot r ayf y psi]
% ayf = yddot + Vx*r

w = 10;

Aol = [A(1,1)      A(1,2)       0   0   0;
       A(2,1)      A(2,2)       0   0   0;
       w*A(1,1)  w*(A(1,2)+Vx) -w   0   0;
       1           0            0   0   0;
       0           1            0   0   0]; 

Bol = [B; w*B(1); 0; 0];

% x
Col = eye(5);

Dol = zeros(5,1);

% open loop poles
disp('Open Loop Poles');
%rifd(eig(Aol))
eig(Aol);
disp(' ');

%% Control Law

C = Col(2,:);  % ayf
CA = C*Aol;
invCB = pinv(C*Bol);

K_r = 8;
K_chi = 4;
K_y = 2;
fi = 0.25;
fc = 0.5;

%% simulation

t1 = 1;
t2 = 7;

theta = 90*pi/180;
thetadot = theta / (t2-t1);
R = Vx/thetadot;
rho = 1/R;

dt = 0.005;
t = (0 : dt : 25)';

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
plot(t, x(:,1)); grid on
ylabel('ydot [m/s]');
subplot(212);
plot(t, x(:,2)*180/pi, t, r_road*180/pi); grid on
ylabel('r, r-road [deg/s]');
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
plot(t, chi*180/pi, t, chi_road*180/pi); grid on
ylabel('chi, chic [deg]');
subplot(212);
plot(t,beta*180/pi); grid on
ylabel('beta [deg]');
xlabel('t [sec]');

figure(4);
subplot(211);
plot(t, N); grid on
ylabel('N [m]');
subplot(212);
plot(t, E); grid on
ylabel('E [m]');
xlabel('t [sec]');

figure(5);
subplot(211);
plot(t, y, t, yc); grid on
ylabel('y, yc [m]');
subplot(212);
plot(t, delta*180/pi); grid on
ylabel('delta [deg]');
xlabel('t [sec]');

figure(6);
plot(E, N); grid
ylabel('N [m]');
xlabel('E [m]');
axis equal
v = axis;
axis([-10 v(2:4)])

