function dy = balrobot(y,m1,m2,r,l,g,u)
%BALROBOT Compute the derivative of the state vector
%   This function computes the first derivative for every state in the
%   state vector.

t_a = (m1 + m2)/(m1 + m2*sin(y(3))^2)*g/l*sin(y(3));
t_b = -(m2*sin(y(3))*cos(y(3)))/(m1+m2*sin(y(3))^2)*y(4)^2;
t_c = (l*m2*cos(y(3)) + r*(m1+m2)) / ((m1+m2*sin(y(3))^2)*l^2*r*m2)*u;

x_a = -(l*m2*sin(y(3)))/(m1+m2)*y(4)^2;
x_b = (l*m2*cos(y(3)))/(m1+m2)*(t_a + t_b + t_c);
x_c = 1/((m1+m2)*r)*u;

dy(1,1) = y(2);
dy(2,1) = x_a + x_b + x_c;
dy(3,1) = y(4);
dy(4,1) = t_a + t_b + t_c;
end