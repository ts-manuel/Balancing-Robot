function [x,theta] = robot_simulation(time, tau, x0, x0_d, theta0, theta0_d, m1, m2, r, l, g)
%ROBOT_NONLINEAR Simulation of the non linear equations of motion
%   The simulation uses the trapezoidal integration method to simulate the
%   non linear differential equation of the system.
%       time = Sample times for witch to evaluate the differential function
%       tau  = Input torque
%       x0   = Initial position
%       x0_d = Initial velocity
%       theta0   = Initial angle
%       theta0_d = Initial angular velocity
%       m1 = Mass of the wheels
%       m2 = Mass of the pendulum
%       r  = Radious of the wheels
%       l  = Distance from the wheels axis to the mass of the pendulum
%       g  = Force of gravity

    x        = zeros(size(time));
    x_d      = zeros(size(time));
    x_dd     = zeros(size(time));
    theta    = zeros(size(time));
    theta_d  = zeros(size(time));
    theta_dd = zeros(size(time));
    
    for i = 1:length(time)
        if i == 1
            x(i)       = x0;         % Initial position
            x_d(i)     = x0_d;       % Initial velocity
            theta(i)   = theta0;     % Initial position
            theta_d(i) = theta0_d;   % Initial angular velocity
            % Solving the differential equation
            theta_dd(i) = compute_theta_dd(tau, theta(i), theta_d(i), m1, m2, r, l, g);
            x_dd(i) = compute_x_dd(tau, theta(i), theta_d(i), theta_dd(i), m1, m2, r, l);
        elseif i == 2
            % Integration
            x_d(i) = x_d(i-1) + x_dd(i-1)*(time(i) - time(i-1));
            x(i) = x(i-1) + x_d(i-1)*(time(i) - time(i-1));
            theta_d(i) = theta_d(i-1) + theta_dd(i-1)*(time(i) - time(i-1));
            theta(i) = theta(i-1) + theta_d(i-1)*(time(i) - time(i-1));
            % Solving the differential equation
            theta_dd(i) = compute_theta_dd(tau, theta(i), theta_d(i), m1, m2, r, l, g);
            x_dd(i) = compute_x_dd(tau, theta(i), theta_d(i), theta_dd(i), m1, m2, r, l);
        else
            % Integration
            x_d(i) = x_d(i-1) + (x_dd(i-2) + x_dd(i-1))/2*(time(i) - time(i-1));
            x(i) = x(i-1) + (x_d(i-2) + x_d(i-1))/2*(time(i) - time(i-1));
            theta_d(i) = theta_d(i-1) + (theta_dd(i-2) + theta_dd(i-1))/2*(time(i) - time(i-1));
            theta(i) = theta(i-1) + (theta_d(i-2) + theta_d(i-1))/2*(time(i) - time(i-1));
            % Solving the differential equation
            theta_dd(i) = compute_theta_dd(tau, theta(i), theta_d(i), m1, m2, r, l, g);
            x_dd(i) = compute_x_dd(tau, theta(i), theta_d(i), theta_dd(i), m1, m2, r, l);
        end
    end
end


function theta_dd = compute_theta_dd(tau, theta, theta_d, m1, m2, r, l, g)
    a = (m1 + m2)/(m1 + m2*sin(theta)^2)*g/l*sin(theta);
    b = -(m2*sin(theta)*cos(theta))/(m1+m2*sin(theta)^2)*theta_d^2;
    c = (l*m2*cos(theta) + r*(m1+m2)) / ((m1+m2*sin(theta)^2)*l^2*r*m2)*tau;

    theta_dd = a + b + c;
end

function x_dd = compute_x_dd(tau, theta, theta_d, theta_dd, m1, m2, r, l)
    a = -(l*m2*sin(theta))/(m1+m2)*theta_d^2;
    b = (l*m2*cos(theta))/(m1+m2)*theta_dd;
    c = 1/((m1+m2)*r)*tau;

    x_dd = a + b + c;
end