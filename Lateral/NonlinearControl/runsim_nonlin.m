clear

mph2fps = 1.47;
fps2mph = 1/mph2fps;
g = 32.2; % fps2

dt = 0.01; % sec
t_start = 0;
t_stop = 20; % sec
tvec = (0:dt:t_stop-dt)';
n = round((t_stop - t_start)/dt);

%% Ego Vehicle

% intial condition
V0 = 40*mph2fps;
chi0 = 0;
N0 = 0;
E0 = 0;

% control law
Kp_N = 1;
Ki_N = 0;

Kp_E = 1;
Ki_E = 0;

intg_Ki_N = V0*cos(chi0);
intg_Ki_E = V0*sin(chi0);

Kp_Ndot = 2.5;
Kp_Edot = 2.5;

Vdot_max = 0.5*g;
Vdot_min = -0.5*g;

chidot_max = 30*pi/180;
chidot_min = -30*pi/180;

V_max = 45*mph2fps;
V_min = 0;

% commands
Nc = zeros(n,1);
Ec = zeros(n,1);
Vc = zeros(n,1);
chic = zeros(n,1);
ac = zeros(n,1);

% go straight
t1 = 2;
n1 = round((t1 - t_start)/dt);
Vc(1:n1) = V0;
chic(1:n1) = chi0;
Nc(1:n1) = my_cumsum(Vc(1:n1))*dt;
Ec(1:n1) = E0;

% lane change
t2 = 7;
n2 = round((t2- t_start)/dt);
Vc(n1+1:n2) = V0;
Nc(n1+1:n2) = my_cumsum(Vc(n1+1:n2))*dt + Nc(n1);
Ec(n1+1:n2) = 24;

% slow down and stop
t3 = 12;
n3 = round((t3 - t_start)/dt);
ac(n2+1:n3) = -V0/(t3-t2);
Vc(n2+1:n3) = my_cumsum(ac(n2+1:n3))*dt + V0;
Nc(n2+1:n3) = my_cumsum(Vc(n2+1:n3))*dt + Nc(n2);
Ec(n2+1:n3) = 24;

Nc(n3+1:end) = Nc(n3);
Ec(n3+1:end) = Ec(n3);


%% Other Vehicle

% intial condition
V01 = 40*mph2fps;
chi01 = 0;
N01 = 150;
E01 = 0;

intg_Ki_N1 = V01*cos(chi01);
intg_Ki_E1 = V01*sin(chi01);

% commands
Nc1 = zeros(n,1);
Ec1 = zeros(n,1);
Vc1 = zeros(n,1);
chic1 = zeros(n,1);
ac1 = zeros(n,1);

% go straight
t11 = 6;
n11 = round((t11 - t_start)/dt);
Vc1(1:n11) = V01;
chic1(1:n11) = chi0;
Nc1(1:n11) = my_cumsum(Vc1(1:n11))*dt + N01;
Ec1(1:n11) = E01;

% slow down and stop
t21 = 13.5;
n21 = round((t21 - t_start)/dt);
Vc1(n11+1:n21) = 0;

ac1(n11+1:n21) = -V01/(t21-t11);
Vc1(n11+1:n21) = my_cumsum(ac1(n11+1:n21))*dt + V01;

Nc1(n11+1:n21) = my_cumsum(Vc1(n11+1:n21))*dt + Nc1(n11);
Ec1(n11+1:n21) = 0;

Nc1(n21+1:end) = Nc1(n21);

%% Simulation

sim('nonlinsim');

%% Get simulation outputs

ego_veh     = getSimVara(N_ego_sim, E_ego_sim, V_ego_sim, chi_ego_sim);
front_veh   = getSimVara(N_front_sim, E_front_sim, V_front_sim, chi_front_sim);
crosss_veh  = getSimVara(N_cross_sim, E_cross_sim, V_cross_sim, chi_cross_sim);


%% Design metric

delN = N2 - N1;


%% Plots

% Plots - Ego

figure(1); clf
subplot(211);
plot(tvec, N_sim(1:end-1), tvec, Nc);
grid on
ylabel('N (ft)');
legend('N', 'Nc');
subplot(212);
plot(tvec, E_sim(1:end-1), tvec, Ec);
grid on
ylabel('E (ft)');
legend('E', 'Ec');
xlabel('t (sec)');
subplot(211);
title('Ego Vehicle');

figure(2); clf
subplot(211);
plot(tvec, V_sim(1:end-1)*fps2mph, tvec, Vc*fps2mph);
grid on
ylabel('V (fps)');
legend('V', 'Vc');
subplot(212);
plot(tvec, chi_sim(1:end-1)*180/pi, tvec, chic*180/pi);
grid on
ylabel('chi (deg)');
legend('chi', 'chic');
xlabel('t (sec)');
subplot(211);
title('Ego Vehicle');

figure(3); clf
subplot(211);
plot(tvec, u_sim(1:end-1,1)/g);
grid on
ylabel('Vdot (g)');
subplot(212);
plot(tvec, u_sim(1:end-1,2)*180/pi);
grid on
ylabel('chidot (deg/s)');
subplot(211);
title('Ego Vehicle');

figure(4); clf
%plot(Ec, Nc); hold on;
plot(E_sim, N_sim)
hold on
plot(E1_sim, N1_sim)
grid on
ylabel('N (ft)');
xlabel('E (ft)');
%legend('Ego','Front');
axis([-100 100 0 1000]);

% Plots - Front

figure(11); clf
subplot(211);
plot(tvec, N1_sim(1:end-1), tvec, Nc1);
grid on
ylabel('N (ft)');
legend('N', 'Nc');
subplot(212);
plot(tvec, E1_sim(1:end-1), tvec, Ec1);
grid on
ylabel('E (ft)');
legend('E', 'Ec');
xlabel('t (sec)');
subplot(211);
title('Front Vehicle');

figure(12); clf
subplot(211);
plot(tvec, V1_sim(1:end-1)*fps2mph, tvec, Vc1*fps2mph);
grid on
ylabel('V (fps)');
legend('V', 'Vc');
subplot(212);
plot(tvec, chi1_sim(1:end-1)*180/pi, tvec, chic1*180/pi);
grid on
ylabel('chi (deg)');
legend('chi', 'chic');
xlabel('t (sec)');
subplot(211);
title('Front Vehicle');

figure(21)
subplot(211);




% draw lanes
figure(4);
hold on

length_straight = 800;

% lane markings
lw = 0.5;
lanemark1.E1 = -6; % ft
lanemark1.E2 = -6; % ft
lanemark1.N1 = 0; % ft
lanemark1.N2 = length_straight; % ft

hold on;
hl = line([lanemark1.E1, lanemark1.E2],[lanemark1.N1, lanemark1.N2]);
set(hl,'linewidth',lw,'color','k')

lanemark2.E1 = 6; % ft
lanemark2.E2 = 6; % ft
lanemark2.N1 = 0; % ft
lanemark2.N2 = length_straight; % ft

hold on;
hl = line([lanemark2.E1, lanemark2.E2],[lanemark2.N1, lanemark2.N2]);
set(hl,'linewidth',lw,'color','k')

lanemark3.E1 = 18; % ft
lanemark3.E2 = 18; % ft
lanemark3.N1 = 0; % ft
lanemark3.N2 = length_straight; % ft

hold on;
hl = line([lanemark3.E1, lanemark3.E2],[lanemark3.N1, lanemark3.N2]);
set(hl,'linewidth',lw,'color','k')

lanemark4.E1 = 30; % ft
lanemark4.E2 = 30; % ft
lanemark4.N1 = 0; % ft
lanemark4.N2 = length_straight; % ft

hold on;
hl = line([lanemark4.E1, lanemark4.E2],[lanemark4.N1, lanemark4.N2]);
set(hl,'linewidth',lw,'color','k')

lanemark4a.thtvec = (pi/2:pi/20:pi);
lanemark4a.R = 10;
lanemark4a.E = lanemark4a.R*cos(lanemark4a.thtvec)+30+10; 
lanemark4a.N = lanemark4a.R*sin(lanemark4a.thtvec)+length_straight;
lanemark4a.E = lanemark4a.E(end:-1:1);    
lanemark4a.N = lanemark4a.N(end:-1:1);

plot(lanemark4a.E, lanemark4a.N, 'linewidth', lw, 'color', 'k')

lanemark5.E1 = -6;
lanemark5.E2 = 30;
lanemark5.N1 = length_straight - 10;
lanemark5.N2 = length_straight - 10;

hl = line([lanemark5.E1, lanemark5.E2],[lanemark5.N1, lanemark5.N2]);
set(hl,'linewidth',lw,'color','k')

lanemark6.E1 = -6;
lanemark6.E2 = 30;
lanemark6.N1 = length_straight;
lanemark6.N2 = length_straight;

hl = line([lanemark6.E1, lanemark6.E2],[lanemark6.N1, lanemark6.N2]);
set(hl,'linewidth',lw,'color','k')
