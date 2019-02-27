
%% settings

clear
close all

mph2fps = 1.47;
fps2mph = 1/mph2fps;
g = 32.2; % fps2

car_length = 15; % ft

dt = 0.01; % sec

v0_ego = 70*mph2fps; % fps
v0_other = v0_ego; % fps
del_v0 = -70*mph2fps;

v_other = v0_other + del_v0; % fps

max_accel = 0.6*g; % fps2
min_accel = -1.0*g; % fps2

safe_dist0_margin = 5; %ft

% Lookup table for safe distance

fit_method = 1;
switch fit_method
    case 1
        v_in = (0:0.1:80); % mph
        d0 = 20;
        slope = 0.6; % (ft/mph)^2
        safe_dist_out = d0 + slope*v_in + safe_dist0_margin; % ft   
        safe_dist0 = d0 + slope*(v0_ego*fps2mph) + safe_dist0_margin;
        
    case 2
        v_in = (0:0.1:100); % mph
        d0 = 20;
        slope = 0.015; % (ft/mph)^2
        safe_dist_out = d0 + slope*v_in.^2 + safe_dist0_margin; % ft
        safe_dist0 = d0 + slope*(v0_ego*fps2mph)^2 + safe_dist0_margin;
end

if true    
    
    figure(101)
    plot(v_in, safe_dist_out - safe_dist0_margin);
    xlabel('v (mph)');
    ylabel('Safe Distance (ft)');
    grid on
    title('Safe Distance (ft)');
    
    figure(102)
    plot(v_in, (safe_dist_out-safe_dist0_margin)/car_length);
    xlabel('v (mph)');
    ylabel(['Safe Distance (x Car Length = ', num2str(car_length), ' ft)']);
    grid on
    title(['Safe Distance (x Car Length = ', num2str(car_length), ' ft)']);
    
    
end

d0_ego = 0; % ft
d0_other = d0_ego + safe_dist0; % ft

%% control law

% w = 4;
% z = 0.7;
% A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 -w^2 -2*z*w];
% B = [0; 0; 0; w^2];
% C = [0 1];
% D = 0;

wa_ego = 4;

A = [0 1; 0 -wa_ego];
B = [0; wa_ego];
C = [0 1];
D = 0;

CA = C*A;
invCB = inv(C*B);

set = 1;

switch set
    case 1
        fc = 0.5;
        fi = 0.25;
        Ka = 8; % -> BW = Kb/2
        Kv = 2;
        Kd = 0.25; % not used for PI control
        
        % sys = 1
        % clp = (s*Kp + Ki)/(S^2 + Kps + Ki)
        % w = sqrt(Ki)
        % z = Kp/(2*sqrt(Ki))
        %
        % Ki = w^2
        % Kp = 2*z*sqrt(Ki) = 2*z*w
        
        w_des = 2;
        z_des = 1.5;
        %Kdi = w_des^2;
        %Kdp = 2*z_des*w_des;

        Kdi = 4;
        Kdp = 8;
        
    case 2
        fc = 0.5;
        fi = 0.25;
        Ka = 8; % -> BW = Kb/2
        Kv = 2;
        Kd = 0.25; % not used for PI control
        
        % sys = 1
        % clp = (s*Kp + Ki)/(S^2 + Kps + Ki)
        % w = sqrt(Ki)
        % z = Kp/(2*sqrt(Ki))
        %
        % Ki = w^2
        % Kp = 2*z*sqrt(Ki) = 2*z*w
        
        w_des = 2.0;
        z_des = 1;
        Kdi = w_des^2;
        Kdp = 2*z_des*w_des;      
        
end


%% simulation

sim('relative_distance_tracking4');


%% plots

figure(1);
plot(time, vel_ego*fps2mph, time, vel_other*fps2mph);
grid on
ylabel('Velocity (mph)');
xlabel('Time (sec)');
title('Velocity (mph)');
legend('ego','other');

figure(2);
plot(time, rel_dist);
hold on
plot(time, (safe_dist - safe_dist0_margin), 'r')
grid on
legend('actual distance','minimum safe distance');
ylabel('Relative Distance (ft)');
xlabel('Time (sec)');
title('Relative Distance (ft)');

figure(3);
plot(time, rel_dist/car_length);
grid on
ylabel(['Relative Distance (x Car Length = ', num2str(car_length), ' ft)']);
xlabel('Time (sec)');
title(['Relative Distance (x Car Length = ', num2str(car_length), ' ft)']);

figure(4);
plot(time, accel_ego/g);
hold on
plot(time,ones(length(time))*0.6,'r')
plot(time,-ones(length(time))*1.0,'r')
grid on
ylabel('Acceleration (g)');
xlabel('Time (sec)');
title('Acceleration (g)');



