function Jac_mat = d_kf_calc_jacobian_h()%t, x, u)
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

g = 9.81;

syms x_s y_s z_s u_n_s v_n_s w_n_s phi_s theta_s psi_s lambda_x lambda_y lambda_z lambda_p lambda_q lambda_r

Jac_mat = jacobian([x_s;
                    y_s;
                    z_s],[x_s y_s z_s w_n_s lambda_x lambda_y lambda_z]);


end

