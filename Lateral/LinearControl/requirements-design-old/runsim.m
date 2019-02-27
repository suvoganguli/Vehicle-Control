clear

mph2fps = 1.47;
fps2mph = 1/mph2fps;

dt = 0.02; % sec

safe_distance_margin = 0;% ft

% scenario = 1 - constaint speed
% scenario = 2 - rear vehicle accelerating

scenario = 2;

switch scenario
    case 1
        
        % initial states
        
        ego_accel = 0;
        ego_vel = 60*mph2fps;
        ego_dist = 0;
        ego_distlat = 0;
        
        front_accel = 0;
        front_vel = 60*mph2fps;
        front_dist = requirement_lon(front_vel);
        front_distlat = 12; % ft
        
        rear_accel = 0;
        rear_vel = 60*mph2fps;
        rear_dist = -requirement_lon(rear_vel);
        rear_distlat = 12; % ft
        
        % commands
        
        rear_distlatc  = rear_distlat;
        ego_distlatc   = ego_distlat + 12; % ft
        front_distlatc = front_distlat;
        
        ego_velc    = ego_vel;
        rear_velc   = rear_vel;
        front_velc  = front_vel;              
        
    case 2
        
        % initial states
        
        ego_accel = 0;
        ego_vel = 60*mph2fps;
        ego_dist = 0;
        ego_distlat = 0;
        
        front_accel = 0;
        front_vel = 60*mph2fps;
        
        front_dist_buffer = 5*mph2fps*3; % ft        
        
        front_safe_distance = requirement_lon(front_vel) + safe_distance_margin;
        front_dist = front_safe_distance + front_dist_buffer;
        front_distlat = 12; % ft
        
        rear_accel = 0;
        rear_vel = 65*mph2fps;
        
        rear_dist_buffer = 5*mph2fps*5; % ft
        rear_safe_distance = requirement_lon(rear_vel) + safe_distance_margin;
        rear_dist =  -rear_safe_distance - rear_dist_buffer;        
        rear_distlat = 12; % ft
        
        % commands
        
        rear_distlatc  = rear_distlat;
        ego_distlatc   = ego_distlat + 12; % ft
        front_distlatc = front_distlat;
        
        ego_velc    = 65*mph2fps;
        rear_velc   = rear_vel;
        front_velc  = front_vel;
        
        rear_vel2c = 60*mph2fps; 
        ego_velc2 = 60*mph2fps;
                
end


%% sim

sim('lane_change_straight');


%% plots

figure(1);
plot(time, ego_y, time, front_y, time, rear_y ,'--');
grid on;
legend('Ego','Front','Rear');
xlabel('Time (sec)');
ylabel('Relative Lateral Distance (ft)');
title('Relative Lateral Distance (ft)');

figure(2);
plot(time, front_x - ego_x, time, ones(1,length(time))*front_safe_distance);
grid on;
xlabel('Time (sec)');
ylabel('Relative LongitudinalDistance  (ft)');
title('Relative Longitudinal Distance - Front Vehicle (ft)');

figure(3);
plot(time, rear_x - ego_x, time, -ones(1,length(time))*rear_safe_distance);
grid on;
xlabel('Time (sec)');
ylabel('Relative Longitudinal Distance (ft)');
title('Relative Longitudinal Distance - Rear Vehicle  (ft)');

figure(4);
plot(time, ego_vel_sim*fps2mph, time, front_vel_sim*fps2mph, time, rear_vel_sim*fps2mph);
grid on
xlabel('Time (sec)');
ylabel('Velocity (mph)');
legend('Ego Vehicle','Front Vehicle','Rear Vehicle');
title('Velocity (mph)');
ylim([55 70]);




