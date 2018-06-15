
function [psi_error] = Vision_error_sim_psi_ga(angle_to_gate,distance_to_gate,gate_pos,pixel_noise,angle_noise)
%use NEU convention (north east up)

f = 168;

x_princip = 153.2;
y_princip = 32;

intr = [f 0  x_princip;
        0  f y_princip ;
        0   0  0  ];

 
cam_pos_x = gate_pos(1)-cos(angle_to_gate)*distance_to_gate;
cam_pos_y = gate_pos(2)-sin(angle_to_gate)*distance_to_gate;
    
cam_pos = [cam_pos_x cam_pos_y gate_pos(3)];
gate_pos;

angles=[deg2rad(0) deg2rad(-20) angle_to_gate];%phi theta psi angles

R = C_b_n( angles(1), angles(2), angles(3) );
R_20 = C_b_n(0, deg2rad(0), 0);

[ gate_points ] = Calc_gate_points_neu( gate_pos );

sim_points_x = zeros(4,1);
sim_points_y = zeros(4,1);
for n = 1:4
    gate_points(:,n);
     [ u_b, v_b ] = proj_point( gate_points(:,n),intr, R_20*R',cam_pos);
     sim_points_x(n) = u_b+(rand(1,1)-0.5)*pixel_noise;
     sim_points_y(n) = v_b+(rand(1,1)-0.5)*pixel_noise;
     
end

phi_n   = angles(1) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);
theta_n = angles(2) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);
psi_n   = angles(3) +(rand(1,1)-0.5)*angle_noise + deg2rad(0);

[selected_pose_av] = pos_from_att(gate_points,cam_pos,sim_points_x-x_princip,sim_points_y-y_princip,phi_n, theta_n, psi_n);


psi_disturbance = deg2rad(0);
[pos_a, phi_est, theta_est, psi_est] = angle_from_pos_ng(gate_pos,[selected_pose_av(1) selected_pose_av(2) cam_pos(3)],sim_points_x-x_princip,sim_points_y-y_princip,-angles(1),-angles(2),0+psi_disturbance);
 

psi_error = angles(3)-psi_est;

