function xdot = d_kf_calc_f_nl(t, x, u)
%UNTITLED Nonlinear state matrix f
%   Detailed explanation goes here

%x1 = x
%x2 = y
%x3 = z
%x4 = w
%x5 = lambda_x
%x6 = lambda_y
%x7 = lambda_z


%u1 = A_x
%u2 = A_y
%u3 = A_z
%u4 = p
%u5 = q
% u6 = phi
% u7 = theta
% u8 = psi

g = 9.81;%
D_x = 1/-0.5;%
D_y = 1/-0.90;%

xdot = zeros(7, 1);

%xdot
xdot(1) = (D_x*(u(1)-x(5))*cos(u(7))+( D_y*(u(2)-x(6))*sin(u(6))+x(4)*cos(u(6)))*sin(u(7)) )*cos(u(8))-( D_y*(u(2)-x(6))*cos(u(6))-x(4)*sin(u(6)))*sin(u(8));
%ydot
xdot(2) = (D_x*(u(1)-x(5))*cos(u(7))+( D_y*(u(2)-x(6))*sin(u(6))+x(4)*cos(u(6)))*sin(u(7)) )*sin(u(8))+( D_y*(u(2)-x(6))*cos(u(6))-x(4)*sin(u(6)))*cos(u(8));
%zdot
xdot(3) = -D_x*(u(1)-x(5))*sin(u(7))+( D_y*(u(2)-x(6))*sin(u(6))+x(4)*cos(u(6)))*cos(u(7)); 
%wdot 
xdot(4) = (u(3)-x(7))+g*cos(u(7))*cos(u(6))+(u(5))*D_x*(u(1)-x(5))-(u(4))*D_y*(u(2)-x(6));
%lambda_x
xdot(5) = 0;%constant
%lambda_y
xdot(6) = 0;
%lambda_z
xdot(7) = 0;

end

