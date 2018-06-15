
function [sim_points_x, sim_points_y, cam_pos] = Gate_angle_search_ng(gate_pos,cam_h,cam_gd,cam_a,phi_g,theta_g,psi_g)

f = 169;
%matlab undistort
intr = [f 0  150.3280;
        0  f 34.8997 ;
        0   0  0  ];
    

R_20 = C_b_n(0, deg2rad(-20), 0);
R = C_b_n( phi_g, theta_g, psi_g );

[ gate_points ] = Calc_gate_points_neu( gate_pos );

sim_points_x = zeros(4,1);
sim_points_y = zeros(4,1);

cam_pos = [gate_pos(1)-cam_gd*cos(cam_a) gate_pos(2)-cam_gd*sin(cam_a) cam_h];

for n = 1:4
    gate_points(:,n);
     [ u_b, v_b ] = proj_point( gate_points(:,n),intr, (R_20*R'),cam_pos);
     sim_points_x(n) = u_b;
     sim_points_y(n) = v_b;
end
