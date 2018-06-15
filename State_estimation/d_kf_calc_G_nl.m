function G = d_kf_calc_G_nl(x,u)
%UNTITLED Nonlinear system noise input matrix
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


D_x = 1/-0.5;%
D_y = 1/-0.43;%
g = 9.81;

G = [D_x*cos(x(6))*cos(x(7))    D_y*sin(x(5))*sin(x(6))*cos(x(7))-D_y*cos(x(5))*sin(x(7))         0   0           0                      0;
     D_x*cos(x(6))*sin(x(7))    D_y*sin(x(5))*sin(x(6))*sin(x(7))+D_y*cos(x(5))*cos(x(7))         0   0           0                      0;
     -D_x*sin(x(6))             D_y*sin(x(5))*cos(x(6))                                           0   0           0                      0;
     u(2)*D_x  -u(4)*D_y 1 -x(5)         x(4)                   0;
     0      0         0  1    sin(x(5))*tan(x(6)) cos(x(5))*tan(x(6));
     0      0         0  0            cos(x(5))          -sin(x(5)) ;
     0      0         0  0    sin(x(5))/cos(x(6)) cos(x(5))/cos(x(6));
     0      0         0   0           0                      0;
     0      0         0   0           0                      0;
     0      0         0   0           0                      0;
     0      0         0   0           0                      0;
     0      0         0   0           0                      0;
     0      0         0   0           0                      0];

end

