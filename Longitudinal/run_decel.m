clear

g = 32.2; %fps2

v0vec = [20 30 45 60 70]; % mph

dt = 0.05; % 20 Hz
t_delay = 1; % sec
z_delay = round(t_delay/dt); % sec

condition = 'normal';

switch condition
    case 'normal'
        decel = 0.6*g; % fps2
        suffix = 'normal';
        tf = 15; % sec
    case 'wet'
        decel = 0.4*g; % fps2
        suffix = 'wet';
        tf = 20; % sec
    case 'icy'
        decel = 0.2*g; % fps2
        suffix = 'icy';     
        tf = 25; % sec
end        

if t_delay >= 3
    suffix = [suffix, '_perception'];
end

k = 1;
for v0 = v0vec
    sim('velocity_tracking_decel');
    filename = ['data\data_', num2str(v0), 'mph_decel_',suffix];
    save(filename,'time','accel','vel','dist');
    braking_dist(k) = dist(end);     %#ok<SAGROW>
    k = k+1;
end

disp([v0vec', round(braking_dist')]);

%% data chart

data_braking = [
20	88	88	107
30	132	141	205
45	211	256	416
60	320	410	702
70	407	534	935
];

figure(1); clf
for k = 2:4
    plot(data_braking(:,1),data_braking(:,k),'o-');
    hold on
end
xlabel('Speed (mph)');
ylabel('Braking Distance (ft)');
legend('normal','wet','dry','Location','NorthWest');
title('Braking Distance');
grid on


data_perception = [
20	147	147	166
30	220	229	293
45	344	389	548
60	497	587	878
70	613	740	1141
];

figure(2); clf
for k = 2:4
    plot(data_perception(:,1),data_perception(:,k),'o-');
    hold on
end
xlabel('Speed (mph)');
ylabel('Perception Distance (ft)');
legend('normal','wet','dry','Location','NorthWest');
title('Perception Distance');
grid on

