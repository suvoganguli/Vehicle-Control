
%% settings

clear

mph2fps = 1.47;
fps2mph = 1/mph2fps;
g = 32.2; % fps2

dt = 0.05; % sec

v0_ego = 0*mph2fps; % fps
v0_other = v0_ego; % fps
del_v0 = 10*mph2fps;

v_other = v0_other + del_v0; % fps

max_accel = 0.6*g; % fps2
min_accel = -0.6*g; % fps2

d0_ego = 0; % ft
del_d0 = 60; % ft
d0_other = d0_ego + del_d0; % ft

%% control law

A = [0 1; 0 -2];
B = [0; 2];
C = [0 1];
D = 0;

CA = C*A;
invCB = inv(C*B);

fc = 0.5;
fi = 0.25;
Ka = 8;
Kv = 4;
Kd = 2;

%% plots

sim('relative_distance_tracking3');

