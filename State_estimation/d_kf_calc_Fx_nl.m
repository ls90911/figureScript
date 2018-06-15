function DFx = d_kf_calc_Fx_nl(Jac_mat, x, u)
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

 x_s = x(1);
 y_s = x(2);
 z_s = x(3);
 w_n_s = x(4);
 lambda_x = x(5);
 lambda_y = x(6);
 lambda_z = x(7);

 A_x = u(1);
 A_y = u(2);
 A_z = u(3);
 p_s = u(4);
 q_s = u(5);
 phi_s = u(6);
 theta_s = u(7);
 psi_s = u(8);
 
 DFx = eval(Jac_mat);

end

