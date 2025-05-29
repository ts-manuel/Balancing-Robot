%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m = 0.46;  % [Kg] Peso del pendolo.
M = 0.08;  % [Kg] Peso delle ruote.
r = 0.04;  % [m]  Raggio delle ruote.
l = 0.048; % [m]  Distanza tra punto di rotazione e baricentro del pendolo.
g = 9.81;  % [m/s^2] Accelerazione di gravità.

a12 = m*g/M;
a32 = (M+m)*g/(M*l);
b1 = 1/(M*r);
b3 = (M*r+m*l)/(M*m*l^2*r);

% a1 = -3*(M+m)*g/(l*(3*m-4*(M+m)));
% b1 = 3*(M+m)/(l*(3*m-4*(M+m)))*(1/(r*(M+m)));
% 
% a2 = -g*3*(M+m)/(3*m*l-7*(M+m)*l);
% b2 = 3*(M+m)/(3*m*l-7*(M+m)*l)/(r*(M+m));

% Sistema in forma di stato
% X0 = x
% X1 = x_punto
% X2 = theta
% X3 = theta_punto

A = [0 1 0 0; 0 0 a12 0; 0 0 0 1; 0 0 a32 0];
B = [0; b1; 0; b2];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% Funzione di trasferimento coppia -> angolo
sys = ss([0 1; a 0], [0; b], [1 0], 0);
rlocus(sys);