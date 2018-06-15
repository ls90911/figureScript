
function [pos_error] = Vision_error_sim_LS(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise)
%close all;

f = 168;
intr = [f 0  153.2;
        0  f 32 ;
        0   0  0  ];

 
cam_pos_x = gate_pos(1)-cos(angle_to_gate)*distance_to_gate;
cam_pos_y = gate_pos(2)-sin(angle_to_gate)*distance_to_gate;
    
cam_pos = [cam_pos_x cam_pos_y gate_pos(3)];

angles=[deg2rad(0) deg2rad(20) angle_to_gate];%phi theta psi angles

R = C_b_n( angles(1), angles(2), angles(3) );

[ gate_points ] = Calc_gate_points_order( gate_pos );

sim_points_x = zeros(4,1);
sim_points_y = zeros(4,1);
for n = 1:4
    gate_points(:,n);
     [ u_b, v_b ] = proj_point( gate_points(:,n),intr, R',cam_pos);
     sim_points_x(n) = u_b+(rand(1,1)-0.5)*pixel_noise-153.2;
     sim_points_y(n) = v_b+(rand(1,1)-0.5)*pixel_noise-32;
     
end

%estimated angles, needed for initial position estimation. 
%add noise  
phi_n   = angles(1) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);
theta_n = angles(2) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);
psi_n   = angles(3) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);

[selected_pose_av] = pos_from_att(gate_points,cam_pos,sim_points_x,sim_points_y,phi_n, theta_n, psi_n);

selected_pose_av;

pos_error = cam_pos-selected_pose_av';


