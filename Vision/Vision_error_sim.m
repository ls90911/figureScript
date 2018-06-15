
function [pos_error] = Vision_error_sim(angle_to_gate,distance_to_gate,gate_pos,pixel_noise)
%close all;

f = 168;

x_princip = 157;
y_princip = 32;

intr = [f 0  x_princip;
        0  f y_princip ;
        0   0  0  ];

 
cam_pos_x = gate_pos(1)-cos(angle_to_gate)*distance_to_gate;
cam_pos_y = gate_pos(2)-sin(angle_to_gate)*distance_to_gate;
    
cam_pos = [cam_pos_x cam_pos_y gate_pos(3)];
gate_pos;

angles=[deg2rad(0) deg2rad(20) angle_to_gate];%phi theta psi angles

R = C_b_n( angles(1), angles(2), angles(3) );

[ gate_points ] = Calc_gate_points_order( gate_pos );

sim_points_x = zeros(4,1);
sim_points_y = zeros(4,1);
for n = 1:4
    gate_points(:,n);
     [ u_b, v_b ] = proj_point( gate_points(:,n),intr, R',cam_pos);
     sim_points_x(n) = u_b+(rand(1,1)-0.5)*pixel_noise;
     sim_points_y(n) = v_b+(rand(1,1)-0.5)*pixel_noise;
     
end

%waitforbuttonpress;


[ selected_pose_av, R ] = pos_p3p_ransac_4(gate_pos, sim_points_x, sim_points_y);
selected_pose_av;

pos_error = cam_pos-selected_pose_av;

Rmat = R';
dcm00 = Rmat(1,1);
dcm01 = Rmat(1,2);
dcm02 = Rmat(1,3);
dcm12 = Rmat(2,3);
dcm22 = Rmat(3,3);

phi_R = rad2deg(atan2(dcm12, dcm22));
theta_R = rad2deg(-asin(dcm02));
psi_R = rad2deg(atan2(dcm01, dcm00));


