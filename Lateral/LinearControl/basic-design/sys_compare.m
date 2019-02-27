
T = 0.01;

V0 = 100; % fps
Chi0 = 30*pi/180*0;
E0 = 10; % ft
N0 = 20; % ft

t = (0:T:100)';
n = length(t);

u1 = zeros(n,1);
u2 = zeros(n,1);

caseno = 2;

switch caseno
    case 1
        u1(1:20) = 10;
        u1(21:30) = -10;
        u2(41:50) = 10*pi/180;
        u2(51:60) = -10*pi/180;
    case 2
        u2(1:n) = 10*pi/180;
end

E_NL = zeros(n,1);
N_NL = zeros(n,1);
V_NL = zeros(n,1);
Chi_NL = zeros(n,1);

E_NL(1) = E0;
N_NL(1) = N0;
V_NL(1) = V0;
Chi_NL(1) = Chi0;

%% Nonlinear

for k = 1:n-1
    E_NL(k+1) = E_NL(k) + T*( V_NL(k) * sin(Chi_NL(k)) );
    N_NL(k+1) = N_NL(k) + T*( V_NL(k) * cos(Chi_NL(k)) );
    V_NL(k+1) = V_NL(k) + T * u1(k);
    Chi_NL(k+1) = Chi_NL(k) + T * u2(k);
end

figure(1); clf
subplot(211);
plot(t,E_NL); ylabel('E [ft]');  grid on;
subplot(212);
plot(t,N_NL); ylabel('N [ft]');  grid on;
xlabel('t [sec]');

figure(2); clf
subplot(211); grid on;
plot(t,V_NL); ylabel('V [fps]');  grid on;
subplot(212);
plot(t,Chi_NL*180/pi); ylabel('Chi [deg]');  grid on;
xlabel('t [sec]');

figure(3); clf
subplot(211);
plot(t,u1); ylabel('u1 = Vdot [fps2]');  grid on;
subplot(212);
plot(t,u2*180/pi); ylabel('u2 = Chidot [deg/s]');  grid on;
xlabel('t [sec]');

%% Linear

method = 2;

switch method
    case 1
        
        x = zeros(4,n);
        x(:,1) = [E0; N0; V0; Chi0];
        
        A_L = [sin(Chi0) V0*cos(Chi0); cos(Chi0) -V0*sin(Chi0)];
        B_L = eye(2);
        
        A = zeros(4,4);
        A(1:2,3:4) = A_L;
        
        B = zeros(4,2);
        B(3:4,1:2) = B_L;
        
        u = [u1 u2];
        
        for k = 1:n-1
            x(:,k+1) = x(:,k) + T*A*x(:,k) + T*B*u(k,:)';
        end

    case 2
        
        x = zeros(4,n);
        x(:,1) = [E0; N0; V0; Chi0];
        
        A_L = [sin(Chi0) V0*cos(Chi0); cos(Chi0) -V0*sin(Chi0)];
        B_L = eye(2);
        
        A = zeros(4,4);
        A(1:2,3:4) = A_L;
        
        B = zeros(4,2);
        B(3:4,1:2) = B_L;
        
        u = [u1 u2];
        
        A_L_old = A_L;
        
        for k = 1:n-1
            x(:,k+1) = x(:,k) + T*A*x(:,k) + T*B*u(k,:)';
            V02 = x(3,k+1);
            Chi02 = x(4,k+1);
            
            A_L = [sin(Chi02) V02*cos(Chi02); cos(Chi02) -V02*sin(Chi02)];
            A(1:2,3:4) = A_L;
            
            if E_NL(k+1) < x(1,k+1)
                %print('diff');
            end
            
       end        
        
end


E_L = x(1,:);
N_L = x(2,:);
V_L = x(3,:);
Chi_L = x(4,:);

figure(1);
subplot(211); hold on
plot(t,E_L,'--');
subplot(212); hold on
plot(t,N_L,'--');

figure(2);
subplot(211); hold on
plot(t,V_L,'--');
subplot(212); hold on
plot(t,Chi_L*180/pi,'--');







