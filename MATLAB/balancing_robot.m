%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear all


m1 = 0.46;  % [Kg] Wheight of the pendulum.
m2 = 0.08;  % [Kg] Wheight of the wheels.
r  = 0.04;  % [m]  Radius of the wheels.
l  = 0.070; % [m]  Distance from the axis of the wheel and the mass of the pendulum.
g  = 9.81;  % [m/s^2] Accelerazione di gravità.



%%
% Simulation of the non linear system by numerically solving the
% differential equation
Ts = 0.0001;    % [s] Sample time
Duration = 2;   % [s] For how long to simulate

x0       = 0;   % Initial position
x0_d     = 0;   % Initial velocity
theta0   = 0.5; % Initial angle
theta0_d = 0;   % Initial angular velocity
tau = 0;

time = 0:Ts:Duration;

% Simulate the non linear differential equation
[x, theta] = robot_simulation(time, tau, x0, x0_d, theta0, theta0_d, m1, m2, r, l, g);

figure();
plot(time, x)
title('Position x')

figure();
plot(time, theta)
title('Angle theta')

% Write simulation to .csv file
% [time; x; theta]
writematrix([time' x' theta'], 'balancing_robot.csv')

%%
% Linearization of the non linear equation of motion at the equilibrium

a12 = m2*g/m1;
a32 = (m1+m2)*g/(m1*l);
b1 = (l+r)/(m1*l*r);
b3 = (l*m2 + r*(m1+m2))/(m1*m2*l^2*r);

% Sistema in forma di stato
% X0 = x
% X1 = x_punto
% X2 = theta
% X3 = theta_punto

A = [0 1 0 0; 0 0 a12 0; 0 0 0 1; 0 0 a32 0];
B = [0; b1; 0; b3];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% Funzione di trasferimento coppia -> angolo
sys = ss([0 1; a32 0], [0; b3], [1 0], 0);
figure();
rlocus(sys);
title('Open Loop Root Locus')

% Controller in the S domain
s = tf('s');
C_s = 4 * (1+0.067*s)*(1+0.067*s)/(s*(1+0.0067*s));
figure();
rlocus(C_s*sys)
title('Close Loop Root Locus')


% Controller in the Z domain
C_z = c2d(C_s, 0.001, 'tustin');
C_z.Variable = 'z^-1';

fprintf('Discrete controller in the Z domain:\n');
tf(C_z)