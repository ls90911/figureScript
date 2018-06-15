function Jac_mat = d_kf_calc_jacobian()%t, x, u)
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

syms x_s y_s z_s u_n_s v_n_s w_n_s phi_s theta_s psi_s lambda_x lambda_y lambda_z lambda_p lambda_q lambda_r A_x A_y A_z p_s q_s r_s

Jac_mat = jacobian([(D_x*(A_x-lambda_x)*cos(theta_s)+( D_y*(A_y-lambda_y)*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*cos(psi_s)-( D_y*(A_y-lambda_y)*cos(phi_s)-w_n_s*sin(phi_s))*sin(psi_s);
(D_x*(A_x-lambda_x)*cos(theta_s)+( D_y*(A_y-lambda_y)*sin(phi_s)+w_n_s*cos(phi_s))*sin(theta_s) )*sin(psi_s)+( D_y*(A_y-lambda_y)*cos(phi_s)-w_n_s*sin(phi_s))*cos(psi_s);
 -D_x*(A_x-lambda_x)*sin(theta_s)+( D_y*(A_y-lambda_y)*sin(phi_s)+w_n_s*cos(phi_s))*cos(theta_s);
  (A_z-lambda_z)+g*cos(theta_s)*cos(phi_s)+(q_s)*D_x*(A_x-lambda_x)-(p_s)*D_y*(A_y-lambda_y);
  0;%constant
  0;
  0],[x_s y_s z_s w_n_s lambda_x lambda_y lambda_z]);


end

