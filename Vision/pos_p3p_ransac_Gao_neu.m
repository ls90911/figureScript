function [ selected_pose_av, Rmat ] = pos_p3p_ransac_Gao_neu(gate_pos, image_points_x, image_points_y)

%for use with the Vision_error_sim.m code 

%intrinsic matrix

f = 168;

x_princip = 153.2;
y_princip = 32;

intr = [f 0  x_princip;
        0  f y_princip ;
        0   0  0  ];
        
[ gate_points ] = Calc_gate_points_neu( gate_pos );

%generate vectors with different combinations of 3 detection points and
% 1 reference point. 
selected_pose = [];

  
 best_error = 1000;
    
for s = 1:4

     gate_shift_points = circshift(gate_points(:,1:4),s,2);
%     
     u_dist = circshift(image_points_x,s);
     v_dist = circshift(image_points_y,s);
     
     gate_points(:,1:4);
     [image_points_x image_points_y];

    res_u = u_dist-x_princip;%
    res_v = v_dist-y_princip;%

    %calc vectors from image points
    [ image_vectors ] = vec_from_image( res_u, res_v,f );% vec_from_image_ned( res_u, res_v,f );
    gate_shift_points ;%(:,1:3)
    [u_dist v_dist];
    image_vectors;
    pose_from_image = opengv('p3p_gao',gate_shift_points(:,1:3),image_vectors);%p3p( gate_shift_points, image_vectors );
    pose_from_image
    temp = size(size(pose_from_image));
    %waitforbuttonpress;
    %select solution by back projection
    error = [];
    for s = 1:4
        %Also check error of first 3 points
        euclid_3 = zeros(3,1);
        for i = 1:3
            [ u_temp, v_temp ] = proj_point( gate_shift_points(:,i),intr, pose_from_image(:,1:3,s)', pose_from_image(:,4,s)' );
            u_ = u_temp-x_princip;%
            v_ = v_temp-y_princip;%
            euclid_3(i) = sqrt((u_ - res_u(i))^2+(v_ - res_v(i))^2);
        end
    
    [ u_temp, v_temp ] = proj_point( gate_shift_points(:,4),intr, pose_from_image(:,1:3,s)', pose_from_image(:,4,s)' );
    rot_error = sum(sum(abs(pose_from_image(:,1:3,s)'-eye(3))));

    u_4 = u_temp-x_princip;%
    v_4 = v_temp-y_princip;%
    index = s;
    euclid = sqrt((u_4 - res_u(4))^2+(v_4 - res_v(4))^2)+sum(euclid_3);
    if(euclid < best_error)
        best_R_p3p = pose_from_image(:,1:3,s)';
        best_pos_p3p = pose_from_image(:,4,s)';
        best_error = euclid;
    end
    pose_from_image(:,s)';
    error = [error;euclid rot_error index];
    
    u_b = zeros(4,1);
    v_b = zeros(4,1);
    %reproject whole gate
    for n = 1:4
     [ u_b(n), v_b(n) ] = proj_point( gate_shift_points(:,n),intr, pose_from_image(:,1:3,s)',pose_from_image(:,4,s)');
    end
   
    end
     
    sorted = sortrows(error);
    selected_pose = [selected_pose; pose_from_image(:,4,sorted(1,3))'];
    
end
%avarage
selected_pose_av = best_pos_p3p;%
%waitforbuttonpress
Rmat = best_R_p3p;

dcm00 = best_R_p3p(1,1);
dcm01 = best_R_p3p(1,2);
dcm02 = best_R_p3p(1,3);
dcm12 = best_R_p3p(2,3);
dcm22 = best_R_p3p(3,3);

phi_R = rad2deg( atan2(dcm12, dcm22));
theta_R = rad2deg( -asin(dcm02));
psi_R = rad2deg(atan2(dcm01, dcm00));

end

