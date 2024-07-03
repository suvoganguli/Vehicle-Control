% Vehicle Dynamics and Controls, Chapter 3, Page 52 - Rajamani

m = 1573; % kg
Iz = 2873; % kg/m^2
a = 1.1; % m
b = 1.58; % m
Cf = 80000; % N/rad
Cr = 80000; % N/rad

Vx = 30; % mps

% x = [e1 e1dot e2 e2dot] 
% e1ddot = (yddot + Vx*r) - Vx^2/R = yddot + Vx(psidot - psidot_des)
% e2 = psi - psidot
% u = delta

A = [0         1                  0            0;
     0   -2*(Cf+Cr)/(m*Vx)  2*(Cf+Cr)/m  -2*(a*Cf-b*Cr)/(m*Vx); 
     0         0                  0            1
     0  -2*(a*Cf-b*Cr)/(Iz*Vx) 2*(a*Cf-b*Cr)/Iz -2*(a^2*Cf+b^2*Cr)/(Iz*Vx)];

B = [0; 2*Cf/m; 0; 2*a*Cf/Iz];

G = [0; -2*(a*Cf-b*Cr)/(m*Vx)-Vx; 0; -2*(a^2*Cf+b^2*Cr)/(Iz*Vx)];  % G*psidot_des

H = [0; 9.81; 0; 0];  % H*sin(phi)
phi = 0;

C = [0 1 0 0];
CA = C*A;
invCB = pinv(C*B);

Kb_e1dot = 8;
Kb_e1 = 2;
fi = 0.25;
fc = 0.5;

x0 = zeros(4,1);


%% 

disp('Open loop poles');
rifd(eig(A)); 
    
model = 'sim_linear_control_error';
warning('off')
[acl,bcl,ccl,dcl] = linmod(model);
warning('on')

disp('Closed loop poles');
rifd(eig(acl)); 


%% simulation

sim(model);




