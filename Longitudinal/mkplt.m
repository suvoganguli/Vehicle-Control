clear
close all

%% Velocity and Acceleration Responses from Zero Speed

filenames = ...
    {'data\data_20mph_accel_normal' 
    'data\data_30mph_accel_normal' 
    'data\data_45mph_accel_normal' 
    'data\data_60mph_accel_normal' 
    'data\data_70mph_accel_normal'};

for k = 1:length(filenames)
    
    load(filenames{k});
    
    figure(1)
    plot(time,vel/1.47);
    xlabel('time (sec)');
    ylabel('velocity (mph)');
    grid on
    hold on
    
    figure(2)
    plot(time,accel/32.2);
    xlabel('time (sec)');
    ylabel('acceleration (g)');
    grid on
    hold on
    
end

figure(1);
legend('20 mph','30 mph','45 mph','60 mph', '70 mph', 'Location', 'NorthWest');

figure(2);
legend('20 mph','30 mph','45 mph','60 mph', '70 mph', 'Location', 'NorthEast');

%% Velocity and Acceleration Responses for Speed Change

load('data\data_20mph_to_45mph_normal');

figure(3)
plot(time,vel/1.47);
xlabel('time (sec)');
ylabel('velocity (mph)');
grid on
hold on

load('data\data_20mph_to_45mph_limits_normal');

figure(3)
plot(time,vel_lo/1.47,time,vel_hi/1.47);
xlabel('time (sec)');
ylabel('velocity (mph)');
legend('ideal response','lower limit','upper limit','Location','SouthEast');
grid on
hold on


%% Deceleration to Stop

filenames = ...
    {'data\data_20mph_decel_normal'
    'data\data_30mph_decel_normal'
    'data\data_45mph_decel_normal'
    'data\data_60mph_decel_normal'
    'data\data_70mph_decel_normal'};

final_distance = [];

for k = 1:length(filenames)
    
    load(filenames{k});
    
    figure(4)
    plot(time,vel/1.47);
    xlabel('time (sec)');
    ylabel('velocity (mph)');
    grid on
    hold on
    
    figure(5)
    plot(time,accel/32.2);
    xlabel('time (sec)');
    ylabel('acceleration (g)');
    grid on
    hold on
    
    figure(6)
    plot(time,dist);
    xlabel('time (sec)');
    ylabel('braking distance (ft)');
    grid on
    hold on
    
    final_distance = [final_distance, dist(end)];
    
end

figure(4);
legend('20 mph','30 mph','45 mph','60 mph', '70 mph', 'Location', 'NorthEast');

figure(5);
legend('20 mph','30 mph','45 mph','60 mph', '70 mph', 'Location', 'SouthEast');

figure(6);
legend('20 mph','30 mph','45 mph','60 mph', '70 mph', 'Location', 'NorthWest');

figure(7);
plot([20 30 45 60 70],final_distance,'o-');
xlabel('speed (mph)');
ylabel('braking distance (ft)');
grid on

%% Low Damping Mode

filename = 'data\data_lowdamp_accel_normal';
load(filename);

figure(8)
plot(time,vel/1.47);
xlabel('time (sec)');
ylabel('velocity (mph)');
grid on
hold on

figure(9)
plot(time,accel/32.2);
xlabel('time (sec)');
ylabel('acceleration (g)');
grid on
hold on



%% Model Basis

load('data\data_60mph_accel_normal')

figure(11)
plot(time,vel/1.47);
xlabel('time (sec)');
ylabel('velocity (mph)');
grid on
hold on

return 

%% Frequency Domain Analysis

sys_throttle2vel = 0.5*tf(2,[1 2 0]);
sys_lowdamp = tf(0.05*5^2*[1 0],[1 2*0.05*5 5^2]);
sys_throttle2vel_lowdamp = tf(2,[1 2 0])*(tf(0.5,1) + sys_lowdamp);
figure(10)
bodemag(sys_throttle2vel, sys_throttle2vel_lowdamp);
legend('nominal','low-damp mode')
title('Open-Loop Bode Magnitude from Velocity Command to Velocity');










