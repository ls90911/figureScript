function [pos_a, phi_est, theta_est, psi_est] = angle_from_pos_grad_2(gate_position,cam_position,sim_points_x,sim_points_y,phi_gate,theta_gate,psi_gate)

%not use ned
gate_pos = gate_position;
cam_pos  = cam_position;


cam_gd = sqrt((gate_pos(1)-cam_pos(1))^2+(gate_pos(2)-cam_pos(2))^2);

cam_a = atan((gate_pos(2)-cam_pos(2))/(gate_pos(1)-cam_pos(1)));
phi_g =  phi_gate;%positive rotation left ccw
theta_g = theta_gate;%positive down
psi_g = psi_gate;%positive to the left

g_step = deg2rad(0.1);%
g_factor = 0.001;

n_iter = 100;%
best_fit = 100000;

[fit, pos_a] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a,phi_g,theta_g,psi_g);

    
for cam_a_i = (cam_a-deg2rad(35)):deg2rad(2):(cam_a+deg2rad(35))
angle_iter = rad2deg(cam_a_i);
cam_a = cam_a_i;
[fit, pos_a] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g);
for i = 1:n_iter
  
    %explore area around previous fit
    [fit_phi, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g+g_step,theta_g,psi_g);
    [fit_theta, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g+g_step,psi_g);
    [fit_psi, ~] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g+g_step);
    
    %gradient at current solution
    gradient = fit-[0 fit_phi fit_theta fit_psi];
    
    %new fit
    %cam_a = cam_a+gradient(1)*g_factor;
    phi_g = phi_g+gradient(2)*g_factor;
    theta_g = theta_g+gradient(3)*g_factor;
    psi_g = psi_g+gradient(4)*g_factor;
    [fit, pos_a] = get_pix_error(sim_points_x,sim_points_y,gate_position,cam_pos,cam_gd,cam_a_i,phi_g,theta_g,psi_g);
   
         
%      waitforbuttonpress

    if(fit < best_fit)
        best_fit = fit;
        rad2deg(cam_a);
        
        %save best results
        phi_est = phi_g;
        theta_est = theta_g;
        psi_est = psi_g;
        rad2deg(psi_est)
        pos_a(3) = -pos_a(3);
        angle_iter = rad2deg(cam_a_i)
        %waitforbuttonpress
    end
    
%     %%%%PLOTTING DEBUGGING
%     %waitforbuttonpress
%      best_fit

end
%waitforbuttonpress

end
best_fit
 %waitforbuttonpress
%     phi_est = phi_g;
%     theta_est = theta_g;
%     psi_est = psi_g;
% 
% pos_a(3) = -pos_a(3);
end

function [error, p_a] = get_pix_error(sim_p_x,sim_p_y,gate_p,cam_p,cam_gate_dist,c_a,phi_,theta_,psi_)
     [search_points_x, search_points_y, p_a] = Gate_angle_search_ng(gate_p,cam_p(3),cam_gate_dist,c_a,phi_,theta_,psi_);
     
    pix_error = zeros(4,1);
    for n = 1:4
       pix_error(n) = sqrt((sim_p_x(n)-search_points_x(n)+150.3280)^2+(sim_p_y(n)-search_points_y(n)+34.8997)^2);
    end

    error = sum(pix_error);%
end
